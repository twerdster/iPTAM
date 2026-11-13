// Copyright 2008 Isis Innovation Limited
#include "PatchFinder.h"
#include "SmallMatrixOpts.h"
#include "KeyFrame.h"

#include "vision.h"
#include "vector_image_ref.h"
#include "image_interpolate.h"
#include "Cholesky.h"
#ifdef IPTAM_MOBILE_PAPER
#include <array>
#endif
// tmmintrin.h contains SSE3<> instrinsics, used for the ZMSSD search at the bottom..
// If this causes problems, just do #define CVD_HAVE_XMMINTRIN 0
#if CVD_HAVE_XMMINTRIN
#include <tmmintrin.h>
#endif

using namespace CVD;
using namespace std;

PatchFinder::PatchFinder(int nPatchSize)
  : mimTemplate(ImageRef(nPatchSize,nPatchSize))
{
  mnPatchSize = nPatchSize;
  mbTemplateBad = true;
  mirCenter = ImageRef(nPatchSize/2, nPatchSize/2);
  int nMaxSSDPerPixel = 500; // Pretty arbitrary... could make a GVar out of this.
  mnMaxSSD = mnPatchSize * mnPatchSize * nMaxSSDPerPixel;
  // Populate the speed-up caches with bogus values:
  mm2LastWarpMatrix = 9999.9 * Identity;
  mpLastTemplateMapPoint = NULL;
};


// Find the warping matrix and search level
int PatchFinder::CalcSearchLevelAndWarpMatrix(MapPoint &p,
					      SE3<> se3CFromW,
					      Matrix<2> &m2CamDerivs)
{
  // Calc point pos in new view camera frame
  // Slightly dumb that we re-calculate this here when the tracker's already done this!
  Vector<3> v3Cam = se3CFromW * p.v3WorldPos;
  if(!std::isfinite(v3Cam*v3Cam) || v3Cam[2] <= 0.001) { mbTemplateBad=true; return -1; }
  double dOneOverCameraZ = 1.0 / v3Cam[2];
  // Project the source keyframe's one-pixel-right and one-pixel-down vectors into the current view
  Vector<3> v3MotionRight = se3CFromW.get_rotation() * p.v3PixelRight_W;
  Vector<3> v3MotionDown = se3CFromW.get_rotation() * p.v3PixelDown_W;
  // Calculate in-image derivatives of source image pixel motions:
  mm2WarpInverse.T()[0] = m2CamDerivs * (v3MotionRight.slice<0,2>() - v3Cam.slice<0,2>() * v3MotionRight[2] * dOneOverCameraZ) * dOneOverCameraZ;
  mm2WarpInverse.T()[1] = m2CamDerivs * (v3MotionDown.slice<0,2>() - v3Cam.slice<0,2>() * v3MotionDown[2] * dOneOverCameraZ) * dOneOverCameraZ;
  double dDet = mm2WarpInverse[0][0] * mm2WarpInverse[1][1] - mm2WarpInverse[0][1] * mm2WarpInverse[1][0];
  mnSearchLevel = 0;
  
  // This warp matrix is likely not appropriate for finding at level zero, which is 
  // the level at which it has been calculated. Vary the search level until the 
  // at that level would be appropriate (does not actually modify the matrix.)
  while(dDet > 3 && mnSearchLevel < 3) // Legacy adaptive search has four active levels.
    {
      mnSearchLevel++;
      dDet *= 0.25;
    };
  
  // Some warps are inappropriate, e.g. too near the camera, too far, or reflected, 
  // or zero area.. reject these!
  if(!std::isfinite(dDet) || dDet > 3 || dDet < 0.25)
    {
      mbTemplateBad = true;
      return -1;
    }
  else
    return mnSearchLevel;
}

// This is just a convenience function wich caluclates the warp matrix and generates
// the template all in one call.
void PatchFinder::MakeTemplateCoarse(MapPoint &p,
				     SE3<> se3CFromW,
				     Matrix<2> &m2CamDerivs)
{
  if(CalcSearchLevelAndWarpMatrix(p, se3CFromW, m2CamDerivs)<0) return;
  MakeTemplateCoarseCont(p);
};

// This function generates the warped search template.
void PatchFinder::MakeTemplateCoarseCont(MapPoint &p)
{
  // Get the warping matrix appropriate for use with CVD::transform...
  Matrix<2> m2 = M2Inverse(mm2WarpInverse) * LevelScale(mnSearchLevel); 
#ifdef IPTAM_MOBILE_PAPER
  int sourceLevel = mbMobileMatching ? mnMobileSourceLevel : p.nSourceLevel;
  Vector<2> sourceCenter = mbMobileMatching ? LevelNPos(LevelZeroPos(p.irCenter,p.nSourceLevel),sourceLevel) : vec(p.irCenter);
  if(mbMobileMatching) m2 *= std::ldexp(1.0,p.nSourceLevel-sourceLevel);
#else
  int sourceLevel=p.nSourceLevel; Vector<2> sourceCenter=vec(p.irCenter);
#endif
  // m2 now represents the number of pixels in the source image for one 
  // pixel of template image
  
  // Optimisation: Don't re-gen the coarse template if it's going to be substantially the 
  // same as was made last time. This saves time when the camera is not moving. For this, 
  // check that (a) this patchfinder is still working on the same map point and (b) the 
  // warping matrix has not changed much.
  
  bool bNeedToRefreshTemplate = mbTemplateBad;
#ifdef IPTAM_MOBILE_PAPER
  if(mbMobileMatching && mnLastMobileSourceLevel!=sourceLevel) bNeedToRefreshTemplate=true;
#endif
  if(&p != mpLastTemplateMapPoint)
    bNeedToRefreshTemplate = true;
  // Still the same map point? Then compare warping matrix..
  for(int i=0; !bNeedToRefreshTemplate && i<2; i++)
    {
      Vector<2> v2Diff = m2.T()[i] - mm2LastWarpMatrix.T()[i];
      const double dRefreshLimit = 0.07;  // Sort of works out as half a pixel displacement in src img
      if(v2Diff * v2Diff > dRefreshLimit * dRefreshLimit)
	bNeedToRefreshTemplate = true;
    }
  
  // Need to regen template? Then go ahead.
  if(bNeedToRefreshTemplate)
    {
      int nOutside;  // Use CVD::transform to warp the patch according the the warping matrix m2
                     // This returns the number of pixels outside the source image hit, which should be zero.
      nOutside = CVD::transform(p.pPatchSourceKF->aLevels[sourceLevel].im,
				mimTemplate, 
				m2,
				sourceCenter,
				vec(mirCenter)); 
      
      if(nOutside)
	mbTemplateBad = true;
      else
	mbTemplateBad = false;
      
      MakeTemplateSums();
      
      // Store the parameters which allow us to determine if we need to re-calculate
      // the patch next time round.
      mpLastTemplateMapPoint = &p;
      mm2LastWarpMatrix = m2;
#ifdef IPTAM_MOBILE_PAPER
      mnLastMobileSourceLevel=sourceLevel;
#endif
    }
};

// This makes a template without warping. Used for epipolar search, where we don't really know 
// what the warping matrix should be. (Although to be fair, I should do rotation for epipolar,
// which we could approximate without knowing patch depth!)
void PatchFinder::MakeTemplateCoarseNoWarp(KeyFrame &k, int nLevel, ImageRef irLevelPos)
{
  mnSearchLevel = nLevel;
  Image<byte> &im = k.aLevels[nLevel].im;
  if(!im.in_image_with_border(irLevelPos, mnPatchSize / 2 + 1))
    {
      mbTemplateBad = true;
      return;
    }
  mbTemplateBad = false;
  copy(im,
       mimTemplate,
       mimTemplate.size(),
       irLevelPos - mirCenter);
  
  MakeTemplateSums();
}

// Convenient wrapper for the above
void PatchFinder::MakeTemplateCoarseNoWarp(MapPoint &p)
{
  MakeTemplateCoarseNoWarp(*p.pPatchSourceKF, p.nSourceLevel,  p.irCenter);
};

// Finds the sum, and sum-squared, of template pixels. These sums are used
// to calculate the ZMSSD.
inline void PatchFinder::MakeTemplateSums()
{
  int nSum = 0;
  int nSumSq = 0;
  ImageRef ir;
  do
    {
      int b = mimTemplate[ir];
      nSum += b;
      nSumSq +=b * b;
    }      
  while(ir.next(mimTemplate.size()));
  mnTemplateSum = nSum;
  mnTemplateSumSq = nSumSq;
}

// One of the main functions of the class! Looks at the appropriate level of 
// the target keyframe to try and find the template. Looks only at FAST corner points
// which are within radius nRange of the center. (Params are supplied in Level0
// coords.) Returns true on patch found.
bool PatchFinder::FindPatchCoarse(ImageRef irPos, KeyFrame &kf, unsigned int nRange)
{
  mbFound = false;
  
  // Convert from L0 coords to search level quantities
  int nLevelScale = LevelScale(mnSearchLevel);
  mirPredictedPos = irPos;
  irPos = irPos / nLevelScale;
  nRange = (nRange + nLevelScale - 1) / nLevelScale;
  
#ifdef IPTAM_MOBILE_PAPER
  if(mbMobileMatching) {
    // Exhaustive integer-pixel circle: independent of target corner detections.
    // Quantized coarse positions can be up to half a target pixel away.
    // Apply the tighter blur acceptance to the refined, unquantized match;
    // retain the normal bound here so an exact fractional match can reach ESM.
    double best=0.65; ImageRef bestPixel;
    auto& image=kf.aLevels[mnSearchLevel].im;
    // Protect mapping callers too: a bounded radius prevents accidental unbounded work.
    int radius=std::min<int>(nRange,32);
    for(int dy=-radius;dy<=radius;++dy) for(int dx=-radius;dx<=radius;++dx) {
      if(dx*dx+dy*dy>radius*radius) continue;
      ImageRef candidate=irPos+ImageRef(dx,dy);
      double score=NormalizedSSDAtPoint(image,candidate);
      if(score<best) {best=score;bestPixel=candidate;mbFound=true;}
    }
    if(mbFound) mv2CoarsePos=LevelZeroPos(bestPixel,mnSearchLevel);
    return mbFound;
  }
#endif
  // Bounding box of search circle
  int nTop = irPos.y - nRange;
  int nBottomPlusOne = irPos.y + nRange + 1;
  int nLeft = irPos.x - nRange;
  int nRight = irPos.x + nRange;
  
  // Ref variable for the search level
  Level &L = kf.aLevels[mnSearchLevel];
  
  // Some bounds checks on the bounding box..
  if(nTop < 0)
    nTop = 0;
  if(nTop >= L.im.size().y)
    return false;
  if(nBottomPlusOne <= 0)
    return false;
  
  // The next section finds all the FAST corners in the target level which 
  // are near enough the search center. It's a bit optimised to use 
  // a corner row look-up-table, since otherwise the routine
  // would spend a long time trawling throught the whole list of FAST corners!
  vector<ImageRef>::iterator i;
  vector<ImageRef>::iterator i_end;
  
  i = L.vCorners.begin() + L.vCornerRowLUT[nTop];
  
  if(nBottomPlusOne >= L.im.size().y)
    i_end = L.vCorners.end();
  else 
    i_end = L.vCorners.begin() + L.vCornerRowLUT[nBottomPlusOne];
  
  ImageRef irBest;             // Best match so far
  int nBestSSD = mnMaxSSD + 1; // Best score so far is beyond the max allowed
  
  for(; i<i_end; i++)          // For each corner ...
    {                         
      if( i->x < nLeft || i->x > nRight)
        continue;
      if((irPos - *i).mag_squared() > nRange * nRange)
	continue;              // ... reject all those not close enough..

      int nSSD;                // .. and find the ZMSSD at those near enough.
      nSSD = ZMSSDAtPoint(L.im, *i);
      if(nSSD < nBestSSD)      // Best yet?
	{
	  irBest = *i;
	  nBestSSD = nSSD;
	}
    } // done looping over corners
  
  if(nBestSSD < mnMaxSSD)      // Found a valid match?
    {
      mv2CoarsePos= LevelZeroPos(irBest, mnSearchLevel);
      mbFound = true;
    }
  else
    mbFound = false;
  return mbFound;
}

// Makes an inverse composition template out of the coarse template.
// Includes calculating image of derivatives (gradients.) The inverse composition
// used here operates on three variables: x offet, y offset, and difference in patch
// means; hence things like mm3HInv are dim 3, but the trivial mean jacobian 
// (always unity, for each pixel) is not stored.
void PatchFinder::MakeSubPixTemplate()
{
#ifdef IPTAM_MOBILE_PAPER
  if(mbMobileMatching) { mv2SubPixPos=mv2CoarsePos; return; }
#endif
  mimJacs.resize(mimTemplate.size() - ImageRef(2,2));
  Matrix<3> m3H = Zeros; // This stores jTj.
  ImageRef ir;
  for(ir.x = 1; ir.x < mnPatchSize - 1; ir.x++)
    for(ir.y = 1; ir.y < mnPatchSize - 1; ir.y++)
      {
	Vector<2> v2Grad;
	v2Grad[0] = 0.5 * (mimTemplate[ir + ImageRef(1,0)] - mimTemplate[ir - ImageRef(1,0)]);
	v2Grad[1] = 0.5 * (mimTemplate[ir + ImageRef(0,1)] - mimTemplate[ir - ImageRef(0,1)]);
	mimJacs[ir-ImageRef(1,1)].first = v2Grad[0];
	mimJacs[ir-ImageRef(1,1)].second = v2Grad[1];
	Vector<3> v3Grad = unproject(v2Grad); // This adds the mean-difference jacobian..
	m3H += v3Grad.as_col() * v3Grad.as_row(); // Populate JTJ.
      }
  
  // Invert JTJ..
  Cholesky<3> chol(m3H);
  if(!std::isfinite(chol.determinant()) || chol.determinant()<=1e-12) {
    mbTemplateBad=true; mm3HInv=Zeros; return;
  }
  mm3HInv = chol.get_inverse();
  // TOON2 Does not have a get_rank for cholesky
  // int nRank = chol.get_rank();
  // if(nRank < 3)
  // cout << "BAD RANK IN MAKESUBPIXELTEMPLATE!!!!" << endl; // This does not happen often (almost never!)
  
  mv2SubPixPos = mv2CoarsePos; // Start the sub-pixel search at the result of the coarse search..
  mdMeanDiff = 0.0;
}

// Iterate inverse composition until convergence. Since it should never have 
// to travel more than a pixel's distance, set a max number of iterations; 
// if this is exceeded, consider the IC to have failed.
bool PatchFinder::IterateSubPixToConvergence(KeyFrame &kf, int nMaxIts)
{
  if(mbTemplateBad) return false;
  const double dConvLimit = 0.03;
  bool bConverged = false;
  int nIts;
  for(nIts = 0; nIts < nMaxIts && !bConverged; nIts++)
    {
      double dUpdateSquared = IterateSubPix(kf);
      if(!std::isfinite(dUpdateSquared) || dUpdateSquared < 0) // went off edge of image
	return false;
      if(dUpdateSquared < dConvLimit*dConvLimit) {
#ifdef IPTAM_MOBILE_PAPER
        if(mbMobileMatching) return MobileSubPixAccepted(kf);
#endif
        return true;
      }
    }
  return false;
}

// Single iteration of inverse composition. This compares integral image positions in the 
// template image to floating point positions in the target keyframe. Interpolation is
// bilinear, and performed manually (rather than using CVD::image_interpolate) since 
// this is a special case where the mixing fractions for each pixel are identical.
double PatchFinder::IterateSubPix(KeyFrame &kf)
{
#ifdef IPTAM_MOBILE_PAPER
  if(mbMobileMatching) return MobileESMIteration(kf);
#endif
  // Search level pos of patch center
  Vector<2> v2Center = LevelNPos(mv2SubPixPos, mnSearchLevel);
  BasicImage<byte> &im = kf.aLevels[mnSearchLevel].im;
  if(!im.in_image_with_border(ir_rounded(v2Center), mnPatchSize / 2 + 1))
    return -1.0;       // Negative return value indicates off edge of image 
  
  // Position of top-left corner of patch in search level
  Vector<2> v2Base = v2Center - vec(mirCenter);
  
  // I.C. JT*d accumulator
  Vector<3> v3Accum = Zeros;
  
  ImageRef ir;
  
  byte* pTopLeftPixel;
  
  // Each template pixel will be compared to an interpolated target pixel
  // The target value is made using bilinear interpolation as the weighted sum
  // of four target image pixels. Calculate mixing fractions:
  double dX = v2Base[0]-floor(v2Base[0]); // Distances from pixel center of TL pixel
  double dY = v2Base[1]-floor(v2Base[1]);
  float fMixTL = (1.0 - dX) * (1.0 - dY);
  float fMixTR = (dX)       * (1.0 - dY);
  float fMixBL = (1.0 - dX) * (dY);
  float fMixBR = (dX)       * (dY);
  
  // Loop over template image
  unsigned long nRowOffset = &kf.aLevels[mnSearchLevel].im[ImageRef(0,1)] - &kf.aLevels[mnSearchLevel].im[ImageRef(0,0)];
  for(ir.y = 1; ir.y < mnPatchSize - 1; ir.y++)
    {
      pTopLeftPixel = &im[::ir(v2Base) + ImageRef(1,ir.y)]; // n.b. the x=1 offset, as with y
      for(ir.x = 1; ir.x < mnPatchSize - 1; ir.x++)
	{
	  float fPixel =   // Calc target interpolated pixel
	    fMixTL * pTopLeftPixel[0]          + fMixTR * pTopLeftPixel[1] + 
	    fMixBL * pTopLeftPixel[nRowOffset] + fMixBR * pTopLeftPixel[nRowOffset + 1];
	  pTopLeftPixel++;
	  double dDiff = fPixel - mimTemplate[ir] + mdMeanDiff;
	  v3Accum[0] += dDiff * mimJacs[ir - ImageRef(1,1)].first;
	  v3Accum[1] += dDiff * mimJacs[ir - ImageRef(1,1)].second;
	  v3Accum[2] += dDiff;  // Update JT*d
	};
    }
  
  // All done looping over image - find JTJ^-1 * JTd:
  Vector<3> v3Update = mm3HInv * v3Accum; 
  mv2SubPixPos -= v3Update.slice<0,2>() * LevelScale(mnSearchLevel);
  mdMeanDiff -= v3Update[2];
  
  double dPixelUpdateSquared = v3Update.slice<0,2>() * v3Update.slice<0,2>();
  return dPixelUpdateSquared;
}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// 
//
//              ZMSSDatpoint, which is SSE optimised, follows
//
// The top version is the SSE version for 8x8 patches. It is compiled
// only if CVD_HAVE_XMMINTRIN is true, also you need to give your 
// compiler the appropriate flags (e.g. -march=core2 -msse3 for g++.)
// The standard c++ version, which is about half as quick (not a disaster
// by any means) is below.
//
// The 8x8 SSE version looks long because it has been unrolled, 
// it just does the same thing eight times. Both versions are one-pass
// and need pre-calculated template sums and sum-squares.
//
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

#if CVD_HAVE_XMMINTRIN
// Horizontal sum of uint16s stored in an XMM register
inline int SumXMM_16(__m128i &target)
{
  unsigned short int sums_store[8];    
  _mm_storeu_si128((__m128i*)sums_store, target);
  return sums_store[0] + sums_store[1] + sums_store[2] + sums_store[3] +
    sums_store[4] + sums_store[5] + sums_store[6] + sums_store[7];
}
// Horizontal sum of uint32s stored in an XMM register
inline int SumXMM_32(__m128i &target)
{
  unsigned int sums_store[4];    
  _mm_storeu_si128((__m128i*)sums_store, target);
  return sums_store[0] + sums_store[1] + sums_store[2] + sums_store[3];
}
#endif

// Calculate the Zero-mean SSD of the coarse patch and a target imate at a specific 
// point.
int PatchFinder::ZMSSDAtPoint(CVD::BasicImage<CVD::byte> &im, const CVD::ImageRef &ir)
{
  if(!im.in_image_with_border(ir, mirCenter[0]))
    return mnMaxSSD + 1;
  
  ImageRef irImgBase = ir - mirCenter;
  byte *imagepointer;
  byte *templatepointer;
  
  int nImageSumSq = 0;
  int nImageSum = 0;
  int nCrossSum = 0;

#if CVD_HAVE_XMMINTRIN
  if(mnPatchSize == 8)
    {
      long unsigned int imagepointerincrement;

      __m128i xImageAsEightBytes;
      __m128i xImageAsWords;
      __m128i xTemplateAsEightBytes;
      __m128i xTemplateAsWords;
      __m128i xZero;
      __m128i xImageSums; // These sums are 8xuint16
      __m128i xImageSqSums; // These sums are 4xint32
      __m128i xCrossSums;   // These sums are 4xint32
      __m128i xProduct;

      
      xImageSums = _mm_setzero_si128();
      xImageSqSums = _mm_setzero_si128();
      xCrossSums = _mm_setzero_si128();
      xZero = _mm_setzero_si128();
      
      imagepointer = &im[irImgBase + ImageRef(0,0)];
      templatepointer = &mimTemplate[ImageRef(0,0)];
      imagepointerincrement = &im[irImgBase + ImageRef(0,1)] - imagepointer;
      
      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      imagepointer += imagepointerincrement;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsEightBytes=_mm_load_si128((__m128i*) templatepointer);
      templatepointer += 16;
      xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);
      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      imagepointer += imagepointerincrement;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      imagepointer += imagepointerincrement;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsEightBytes=_mm_load_si128((__m128i*) templatepointer);
      templatepointer += 16;
      xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);
      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      imagepointer += imagepointerincrement;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      imagepointer += imagepointerincrement;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsEightBytes=_mm_load_si128((__m128i*) templatepointer);
      templatepointer += 16;
      xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);
      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      imagepointer += imagepointerincrement;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      imagepointer += imagepointerincrement;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsEightBytes=_mm_load_si128((__m128i*) templatepointer);
      templatepointer += 16;
      xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);
      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

      nImageSum = SumXMM_16(xImageSums);
      nCrossSum = SumXMM_32(xCrossSums);
      nImageSumSq = SumXMM_32(xImageSqSums);
    }
  else
#endif 
    {    
      for(int nRow = 0; nRow < mnPatchSize; nRow++)
	{
	  imagepointer = &im[irImgBase + ImageRef(0,nRow)];
	  templatepointer = &mimTemplate[ImageRef(0,nRow)];
	  for(int nCol = 0; nCol < mnPatchSize; nCol++)
	    {
	      int n = imagepointer[nCol];
	      nImageSum += n;
	      nImageSumSq += n*n;
	      nCrossSum += n * templatepointer[nCol];
	    };
	}
    };
  
  int SA = mnTemplateSum;
  int SB = nImageSum;
  
  int N = mnPatchSize * mnPatchSize;
  return ((2*SA*SB - SA*SA - SB*SB)/N + nImageSumSq + mnTemplateSumSq - 2*nCrossSum);
}







#ifdef IPTAM_MOBILE_PAPER
void PatchFinder::MakeMobileTemplateAtRoot(KeyFrame& frame,int level,const Vector<2>& rootPosition) {
  mbTemplateBad=true;
  if(level<0||level>=LEVELS||!std::isfinite(rootPosition*rootPosition)) return;
  mnSearchLevel=level;mbMobileMatching=true;
  Matrix<2> identity=Identity;
  Vector<2> center=LevelNPos(rootPosition,level);
  int outside=CVD::transform(frame.aLevels[level].im,mimTemplate,identity,center,vec(mirCenter));
  if(outside) return;
  mbTemplateBad=false;mpLastTemplateMapPoint=nullptr;mnLastMobileSourceLevel=-1;
  MakeTemplateSums();
}

int PatchFinder::CalcSearchLevelAndWarpMatrixAtLevel(MapPoint& point,SE3<> pose,Matrix<2>& cameraDerivative,int targetLevel) {
  // Obtain the continuous source-pixel -> root-target warp. The legacy routine
  // may reject its preferred target level; the mobile source-level choice below
  // is independently checked and does not use that rejection decision.
  CalcSearchLevelAndWarpMatrix(point,pose,cameraDerivative);
  Vector<3> cameraPoint=pose*point.v3WorldPos;
  double determinant=mm2WarpInverse[0][0]*mm2WarpInverse[1][1]-mm2WarpInverse[0][1]*mm2WarpInverse[1][0];
  double energy=0; for(int r=0;r<2;++r) for(int c=0;c<2;++c) energy+=mm2WarpInverse[r][c]*mm2WarpInverse[r][c];
  if(!std::isfinite(cameraPoint*cameraPoint)||cameraPoint[2]<=.001||!std::isfinite(energy)||!std::isfinite(determinant)||determinant<=1e-10||energy/determinant>12||targetLevel<0||targetLevel>=LEVELS) {mbTemplateBad=true;return -1;}
  double scale=std::sqrt(determinant);
  int preferred=static_cast<int>(std::lround(point.nSourceLevel+targetLevel-std::log2(scale)));
  mnMobileSourceLevel=std::max(0,std::min(LEVELS-1,preferred));
  double adjusted=scale*std::ldexp(1.,mnMobileSourceLevel-point.nSourceLevel-targetLevel);
  if(adjusted<.5||adjusted>2.0||!point.pPatchSourceKF||point.pPatchSourceKF->aLevels[mnMobileSourceLevel].im.size().x<mnPatchSize+2) {mbTemplateBad=true;return -1;}
  mnSearchLevel=targetLevel; return targetLevel;
}

double PatchFinder::NormalizedSSDAtPoint(BasicImage<byte>& image,const ImageRef& center) const {
  if(mbTemplateBad||!image.in_image_with_border(center,mnPatchSize/2+1)) return 2.;
  double sum=0,sq=0,cross=0; int count=mnPatchSize*mnPatchSize;
  ImageRef base=center-mirCenter;
  for(int y=0;y<mnPatchSize;++y) for(int x=0;x<mnPatchSize;++x) {
    double value=image[base+ImageRef(x,y)]; sum+=value;sq+=value*value;cross+=value*mimTemplate[y][x];
  }
  double vt=mnTemplateSumSq-double(mnTemplateSum)*mnTemplateSum/count;
  double vi=sq-sum*sum/count;
  if(vt<count*4.||vi<count*4.) return 2.;
  double correlation=(cross-double(mnTemplateSum)*sum/count)/std::sqrt(vt*vi);
  return std::isfinite(correlation) ? 2.*(1.-std::max(-1.,std::min(1.,correlation))) : 2.;
}

namespace {
double MobilePixel(BasicImage<byte>& image,double x,double y) {
  int ix=static_cast<int>(std::floor(x)),iy=static_cast<int>(std::floor(y));
  double ax=x-ix,ay=y-iy;
  return (1-ay)*((1-ax)*image[iy][ix]+ax*image[iy][ix+1])+ay*((1-ax)*image[iy+1][ix]+ax*image[iy+1][ix+1]);
}
}
double PatchFinder::MobileESMIteration(KeyFrame& frame) {
  Vector<2> center=LevelNPos(mv2SubPixPos,mnSearchLevel);
  auto& image=frame.aLevels[mnSearchLevel].im;
  auto inside=[&](const Vector<2>& p) {
    return std::isfinite(p*p)&&p[0]>=mnPatchSize/2+2&&p[1]>=mnPatchSize/2+2&&p[0]<image.size().x-mnPatchSize/2-2&&p[1]<image.size().y-mnPatchSize/2-2;
  };
  if(!inside(center)) return -1.;
  if(mnPatchSize!=8) return -1.; // The mobile paper profile specifies 8x8 patches.
  const int n=(mnPatchSize-2)*(mnPatchSize-2);
  struct Sample {double t,i;Vector<2> gt,gi;};std::array<Sample,36> samples;int sampleIndex=0;
  double meanT=0,meanI=0,sqT=0,sqI=0;Vector<2> meanGT=Zeros,meanGI=Zeros;
  for(int y=1;y<mnPatchSize-1;++y) for(int x=1;x<mnPatchSize-1;++x) {
    double xx=center[0]+x-mirCenter.x,yy=center[1]+y-mirCenter.y;
    Sample sample;sample.t=mimTemplate[y][x];sample.i=MobilePixel(image,xx,yy);
    sample.gt=makeVector(.5*(mimTemplate[y][x+1]-mimTemplate[y][x-1]),.5*(mimTemplate[y+1][x]-mimTemplate[y-1][x]));
    sample.gi=makeVector(.5*(MobilePixel(image,xx+1,yy)-MobilePixel(image,xx-1,yy)),.5*(MobilePixel(image,xx,yy+1)-MobilePixel(image,xx,yy-1)));
    meanT+=sample.t;meanI+=sample.i;sqT+=sample.t*sample.t;sqI+=sample.i*sample.i;
    meanGT+=sample.gt;meanGI+=sample.gi;samples[sampleIndex++]=sample;
  }
  meanT/=n;meanI/=n;meanGT/=n;meanGI/=n;
  double sdT=std::sqrt(std::max(0.,sqT/n-meanT*meanT)),sdI=std::sqrt(std::max(0.,sqI/n-meanI*meanI));
  if(sdT<2||sdI<2) return -1.;
  Vector<2> contrastT=Zeros,contrastI=Zeros;
  for(const auto& sample:samples) {contrastT+=(sample.t-meanT)*sample.gt;contrastI+=(sample.i-meanI)*sample.gi;}
  contrastT/=n*sdT*sdT;contrastI/=n*sdI*sdI;
  Matrix<2> hessian=Zeros;Vector<2> rhs=Zeros;double oldCost=0;
  for(const auto& sample:samples) {
    // Differentiating the mean and contrast normalization matters at coarse
    // half-pixel phases. ESM averages both normalized image gradients.
    const Vector<2> gradient=.5*((sample.gt-meanGT-(sample.t-meanT)*contrastT)/sdT+(sample.gi-meanGI-(sample.i-meanI)*contrastI)/sdI);
    double residual=(sample.i-meanI)/sdI-(sample.t-meanT)/sdT;
    hessian+=gradient.as_col()*gradient.as_row();rhs+=gradient*residual;oldCost+=residual*residual;
  }
  double determinant=hessian[0][0]*hessian[1][1]-hessian[0][1]*hessian[1][0];
  double trace=hessian[0][0]+hessian[1][1];
  if(!std::isfinite(determinant)||determinant<1e-8||trace*trace/determinant>1e5) return -1.;
  Vector<2> update=M2Inverse(hessian)*rhs;double norm2=update*update;
  if(!std::isfinite(norm2)||norm2>2.25) return -1.;
  for(double fraction=1.;fraction>=1./64.;fraction*=.5) {
    const Vector<2> trial=center-update*fraction;if(!inside(trial)) continue;
    double sum=0,square=0;std::array<double,36> values;int valueIndex=0;
    for(int y=1;y<mnPatchSize-1;++y) for(int x=1;x<mnPatchSize-1;++x) {
      double value=MobilePixel(image,trial[0]+x-mirCenter.x,trial[1]+y-mirCenter.y);
      values[valueIndex++]=value;sum+=value;square+=value*value;
    }
    double mean=sum/n,sd=std::sqrt(std::max(0.,square/n-mean*mean));if(sd<2.) continue;
    double cost=0;for(int i=0;i<n;++i) {double residual=(values[i]-mean)/sd-(samples[i].t-meanT)/sdT;cost+=residual*residual;}
    if(std::isfinite(cost)&&cost<=oldCost+1e-10) {
      mv2SubPixPos-=update*(fraction*LevelScale(mnSearchLevel));return norm2*fraction*fraction;
    }
  }
  return norm2<.03*.03 ? norm2 : -1.;
}

bool PatchFinder::MobileSubPixAccepted(KeyFrame& frame) const {
  Vector<2> center=LevelNPos(mv2SubPixPos,mnSearchLevel);
  auto& image=frame.aLevels[mnSearchLevel].im;
  if(!std::isfinite(center*center)||center[0]<mnPatchSize/2+1||center[1]<mnPatchSize/2+1||center[0]>=image.size().x-mnPatchSize/2-1||center[1]>=image.size().y-mnPatchSize/2-1) return false;
  Vector<2> movement=(mv2SubPixPos-mv2CoarsePos)/LevelScale(mnSearchLevel);
  if(movement*movement>4.) return false;
  double sum=0,sq=0,cross=0;const int n=mnPatchSize*mnPatchSize;
  for(int y=0;y<mnPatchSize;++y) for(int x=0;x<mnPatchSize;++x) {
    double pixel=MobilePixel(image,center[0]+x-mirCenter.x,center[1]+y-mirCenter.y);
    sum+=pixel;sq+=pixel*pixel;cross+=pixel*mimTemplate[y][x];
  }
  double vi=sq-sum*sum/n,vt=mnTemplateSumSq-double(mnTemplateSum)*mnTemplateSum/n;
  if(vi<n*4.||vt<n*4.) return false;
  double correlation=(cross-double(mnTemplateSum)*sum/n)/std::sqrt(vi*vt);
  return std::isfinite(correlation)&&2.*(1.-correlation)<mdMobileMaxNSSD;
}

#endif
