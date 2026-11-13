#pragma once
// Window/rendering compatibility only. Original TrackFrame is called bDraw=false.
enum { GL_POINTS, GL_LINE_SMOOTH, GL_BLEND, GL_SRC_ALPHA,
       GL_ONE_MINUS_SRC_ALPHA, GL_LINE_STRIP, GL_POINT_SMOOTH, GL_LINES };
inline void glColor3f(float, float, float) {}
inline void glColor4f(float, float, float, float) {}
inline void glPointSize(float) {}
inline void glLineWidth(float) {}
inline void glBegin(int) {}
inline void glEnd() {}
inline void glEnable(int) {}
inline void glDisable(int) {}
inline void glBlendFunc(int, int) {}
template<class T> inline void glDrawPixels(const T&) {}
template<class T> inline void glVertex(const T&) {}
template<class T> inline void glColor(const T&) {}
