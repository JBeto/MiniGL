/**
 * minigl.cpp
 * -------------------------------
 * Implement miniGL here.
 *
 * You may include minigl.h and any of the standard C++ libraries.
 * No other includes are permitted.  Other preprocessing directives
 * are also not permitted.  These requirements are strictly
 * enforced.  Be sure to run a test grading to make sure your file
 * passes the sanity tests.
 *
 * The behavior of the routines your are implenting is documented here:
 * https://www.opengl.org/sdk/docs/man2/
 * Note that you will only be implementing a subset of this.  In particular,
 * you only need to implement enough to pass the tests in the suite.
 */

#include "minigl.h"
#include "vec.h"
#include "mat.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <vector>
#include <cstdio>
#include <iostream>
#include <vector>
#include <stack>
#include <limits>

using namespace std;

/**
 * Useful data types
 */
typedef mat<MGLfloat,4,4> mat4; //data structure storing a 4x4 matrix, see mat.h
typedef mat<MGLfloat,3,3> mat3; //data structure storing a 3x3 matrix, see mat.h
typedef vec<MGLfloat,4> vec4;   //data structure storing a 4 dimensional vector, see vec.h
typedef vec<MGLfloat,3> vec3;   //data structure storing a 3 dimensional vector, see vec.h
typedef vec<MGLfloat,2> vec2;   //data structure storing a 2 dimensional vector, see vec.h

/**
 * Standard macro to report errors
 */
inline void MGL_ERROR(const char* description) {
    printf("%s\n", description);
    exit(1);
}

vec3 active_color;

deque<mat4> projection_deque(1, mat4({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}));
stack<mat4> projection_stack(projection_deque);
deque<mat4> modelview_deque(1, mat4({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}));
stack<mat4> modelview_stack(modelview_deque);

MGLmatrix_mode matrix_mode = MGL_MODELVIEW;

MGLfloat near_plane = -numeric_limits<float>::max();
MGLfloat far_plane = numeric_limits<float>::max();

struct Vertex {
    vec4 position;
    vec3 color;
    Vertex(vec4 _position, vec3 _color) : position(_position), color(_color) {}
};

struct Triangle {
    Vertex a;
    Vertex b;
    Vertex c;
    Triangle(Vertex _a, Vertex _b, Vertex _c) : a(_a), b(_b), c(_c) {}
    void print() {
        cout << "Triangle: " << a.position << ", " << b.position << ", " << c.position << endl;
    }
};

vector<Vertex> vertices;
vector<Triangle> triangles;
MGLpoly_mode primitive_type;

MGLfloat inline Calculate_Area(vec2 a, vec2 b, vec2 c) {
    return a[0] * (b[1] - c[1]) + a[1] * (c[0] - b[0]) + (b[0] * c[1] - b[1] * c[0]);
}

inline stack<mat4>* getCurrentStack() {
    return matrix_mode == MGL_PROJECTION ? &projection_stack : &modelview_stack;
}

inline mat4* getCurrentMatrix() {
    return &(getCurrentStack()->top());
}

inline void setToIdentity(mat4 &identity) {
    identity.make_zero();
    identity(0, 0) = 1;
    identity(1, 1) = 1;
    identity(2, 2) = 1;
    identity(3, 3) = 1;
}

int inline NDC_To_Pixel_Space(float coord, int length) {
    return (int)(((coord + 1) * length / 2.0) - 0.5);
}

bool inline isInScreen(vec2 p, int width, int height) {
    return (p[0] < width && p[0] >= 0 && p[1] < height && p[1] > 0);
}

int inline minPosition(int index, vec2 a, vec2 b, vec2 c) {
    return max(min(a[index], min(b[index], c[index])), (float)0);
}

int inline maxPosition(int index, float m, vec2 a, vec2 b, vec2 c) {
    return min(max(a[index], max(b[index], c[index])), m);
}

void Rasterize_Triangle(const Triangle& tri, int width, int height, MGLpixel* data, vector<MGLfloat> &zBuffer) {
    vec4 a = tri.a.position;
    vec4 b = tri.b.position;
    vec4 c = tri.c.position;
    
    vec2 a_position(NDC_To_Pixel_Space(a[0] / a[3], width), NDC_To_Pixel_Space(a[1] / a[3], height));
    vec2 b_position(NDC_To_Pixel_Space(b[0] / b[3], width), NDC_To_Pixel_Space(b[1] / b[3], height));
    vec2 c_position(NDC_To_Pixel_Space(c[0] / c[3], width), NDC_To_Pixel_Space(c[1] / c[3], height));
    MGLfloat triangle_area = Calculate_Area(a_position, b_position, c_position);
    
    auto a_z = a[2] / a[3];
    auto b_z = b[2] / b[3];
    auto c_z = c[2] / c[3];

    for (int i = minPosition(0, a_position, b_position, c_position); i <= maxPosition(0, width - 1, a_position, b_position, c_position); i++) {
        for (int j = minPosition(1, a_position, b_position, c_position); j <= maxPosition(1, height - 1, a_position, b_position, c_position); j++) {
            vec2 p_position(i, j);
            // MGLfloat abp_area = Calculate_Area(a_position, b_position, p_position);
            MGLfloat apc_area = Calculate_Area(a_position, p_position, c_position);
            MGLfloat pbc_area = Calculate_Area(p_position, b_position, c_position);
            
            // Screen space
            MGLfloat alpha_prime = pbc_area / triangle_area;
            MGLfloat beta_prime  = apc_area / triangle_area;
            MGLfloat gamma_prime = 1 - alpha_prime - beta_prime;
            
            MGLfloat weighted_alpha = alpha_prime / a[3];
            MGLfloat weighted_beta = beta_prime / b[3];
            MGLfloat weighted_gamma = gamma_prime / c[3];
            MGLfloat sum = weighted_alpha + weighted_beta + weighted_gamma;
            
            // Object space
            MGLfloat alpha = weighted_alpha / sum;
            MGLfloat beta = weighted_beta / sum;
            MGLfloat gamma = weighted_gamma / sum;
            
            if (alpha_prime >= 0 && beta_prime >= 0 && gamma_prime >= 0) {
                auto z = alpha_prime * a_z + beta_prime * b_z + gamma_prime * c_z;
                
                int pixel_index = i + j * width;
                if (z < zBuffer[pixel_index] && z < far_plane && z > near_plane) {
                    zBuffer[pixel_index] = z;
                    vec3 color = alpha * tri.a.color + beta * tri.b.color + gamma * tri.c.color;
                    data[pixel_index] = Make_Pixel(color[0] * 255, color[1] * 255, color[2] * 255);
                }
            }
        }
    }
}

/**
 * Read pixel data starting with the pixel at coordinates
 * (0, 0), up to (width,  height), into the array
 * pointed to by data.  The boundaries are lower-inclusive,
 * that is, a call with width = height = 1 would just read
 * the pixel at (0, 0).
 *
 * Rasterization and z-buffering should be performed when
 * this function is called, so that the data array is filled
 * with the actual pixel values that should be displayed on
 * the two-dimensional screen.
 */
void mglReadPixels(MGLsize width,
                   MGLsize height,
                   MGLpixel *data)
{
    for (unsigned i = 0; i < width; i++) {
        for (unsigned j = 0; j < height; j++) {
            data[i + j * width] = Make_Pixel(0, 0, 0);
        }
    }
    vector<MGLfloat> zBuffer(width * height, numeric_limits<float>::max());
    for(Triangle t : triangles) {
        Rasterize_Triangle(t, width, height, data, zBuffer);
    }

    triangles.clear();
}

// current matrix is on left
/**
 * Start specifying the vertices for a group of primitives,
 * whose type is specified by the given mode.
 */
void mglBegin(MGLpoly_mode mode)
{
    primitive_type = mode;
}

/**
 * Stop specifying the vertices for a group of primitives.
 */
void mglEnd()
{
    if (primitive_type == MGL_TRIANGLES) {
        for (unsigned vi = 0; vi < vertices.size(); vi += 3) {
            Vertex a = vertices[vi];
            Vertex b = vertices[vi + 1];
            Vertex c = vertices[vi + 2];
            triangles.push_back(Triangle(a, b, c));
        }
    }
    else {
        for (unsigned vi = 0; vi < vertices.size(); vi += 4) {
            Vertex a = vertices[vi];
            Vertex b = vertices[vi + 1];
            Vertex c = vertices[vi + 2];
            Vertex d = vertices[vi + 3];
            triangles.push_back(Triangle(a, b, c));
            triangles.push_back(Triangle(a, c, d));
        }
    }
    vertices.clear();
}

/**
 * Specify a two-dimensional vertex; the x- and y-coordinates
 * are explicitly specified, while the z-coordinate is assumed
 * to be zero.  Must appear between calls to mglBegin() and
 * mglEnd().
 */
void mglVertex2(MGLfloat x,
                MGLfloat y)
{
    mglVertex3(x, y, 0);
}

/**
 * Specify a three-dimensional vertex.  Must appear between
 * calls to mglBegin() and mglEnd().
 */
void mglVertex3(MGLfloat x,
                MGLfloat y,
                MGLfloat z)
{
    vec4 position(x, y, z, 1);
    vertices.push_back(Vertex(projection_stack.top() * (modelview_stack.top() * position), active_color));
}

/**
 * Set the current matrix mode (modelview or projection).
 */
void mglMatrixMode(MGLmatrix_mode mode)
{
    matrix_mode = mode;
}

/**
 * Push a copy of the current matrix onto the stack for the
 * current matrix mode.
 */
void mglPushMatrix()
{
    stack<mat4>* currentStack = getCurrentStack();
    mat4 copy = currentStack->top();
    currentStack->push(copy);
}

/**
 * Pop the top matrix from the stack for the current matrix
 * mode.
 */
void mglPopMatrix()
{
    getCurrentStack()->pop();
}

/**
 * Replace the current matrix with the identity.Load
 */
void mglLoadIdentity()
{
    mat4* currentMatrix = getCurrentMatrix();
    setToIdentity(*currentMatrix);
}

/**
 * Replace the current matrix with an arbitrary 4x4 matrix,
 * specified in column-major order.  That is, the matrix
 * is stored as:
 *
 *   ( a0  a4  a8  a12 )
 *   ( a1  a5  a9  a13 )
 *   ( a2  a6  a10 a14 )
 *   ( a3  a7  a11 a15 )
 *
 * where ai is the i'th entry of the array.
 */
void mglLoadMatrix(const MGLfloat *matrix)
{
    mat4* currentMatrix = getCurrentMatrix();
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            (*currentMatrix)(i, j) = matrix[i + (j * 4)];
        }
    }
}

/**
 * Multiply the current matrix by an arbitrary 4x4 matrix,
 * specified in column-major order.  That is, the matrix
 * is stored as:
 *
 *   ( a0  a4  a8  a12 )
 *   ( a1  a5  a9  a13 )
 *   ( a2  a6  a10 a14 )
 *   ( a3  a7  a11 a15 )
 *
 * where ai is the i'th entry of the array.
 */
void mglMultMatrix(const MGLfloat *matrix)
{
    mat4 mult;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            mult(i, j) = matrix[i + (j * 4)];
        }
    }
    mat4* currentMatrix = getCurrentMatrix();
    *currentMatrix = *currentMatrix * mult;
}

/**
 * Multiply the current matrix by the translation matrix
 * for the translation vector given by (x, y, z).
 */
void mglTranslate(MGLfloat x,
                  MGLfloat y,
                  MGLfloat z)
{
    mat4 translationMatrix;
    setToIdentity(translationMatrix);
    translationMatrix(0, 3) = x;
    translationMatrix(1, 3) = y;
    translationMatrix(2, 3) = z;
    
    mat4* currentMatrix = getCurrentMatrix();
    *currentMatrix = *currentMatrix * translationMatrix;
}

/**
 * Multiply the current matrix by the rotation matrix
 * for a rotation of (angle) degrees about the vector
 * from the origin to the point (x, y, z).
 */
void mglRotate(MGLfloat angle,
               MGLfloat x,
               MGLfloat y,
               MGLfloat z)
{
    double radians = angle * 3.14159265 / 180;
    MGLfloat c = cos(radians);
    MGLfloat s = sin(radians);
    vec3 rotateVector(x, y, z);
    vec3 v = rotateVector.normalized();
    
    mat4 rotateMatrix;
    rotateMatrix(0, 0) = (v[0] * v[0] * (1 - c) + c);
    rotateMatrix(0, 1) = (v[0] * v[1] * (1 - c) - v[2] * s);
    rotateMatrix(0, 2) = (v[0] * v[2] * (1 - c) + v[1] * s);
    rotateMatrix(0, 3) = 0;

    rotateMatrix(1, 0) = (v[1] * v[0] * (1 - c) + v[2] * s);
    rotateMatrix(1, 1) = (v[1] * v[1] * (1 - c) + c);
    rotateMatrix(1, 2) = (v[1] * v[2] * (1 - c) - v[0] * s);
    rotateMatrix(1, 3) = 0;

    rotateMatrix(2, 0) = (v[0] * v[2] * (1 - c) - v[1] * s);
    rotateMatrix(2, 1) = (v[1] * v[2] * (1 - c) + v[0] * s);
    rotateMatrix(2, 2) = (v[2] * v[2] * (1 - c) + c);
    rotateMatrix(2, 3) = 0;

    rotateMatrix(3, 0) = 0;
    rotateMatrix(3, 1) = 0;
    rotateMatrix(3, 2) = 0;
    rotateMatrix(3, 3) = 1;

    mat4* currentMatrix = getCurrentMatrix();
    *currentMatrix = *currentMatrix * rotateMatrix;
}

/**
 * Multiply the current matrix by the scale matrix
 * for the given scale factors.
 */
void mglScale(MGLfloat x,
              MGLfloat y,
              MGLfloat z)
{
    mat4 scaleMatrix;
    scaleMatrix.make_zero();
    scaleMatrix(0, 0) = x;
    scaleMatrix(1, 1) = y;
    scaleMatrix(2, 2) = z;
    scaleMatrix(3, 3) = 1;
    
    mat4* currentMatrix = getCurrentMatrix();
    *currentMatrix = *currentMatrix * scaleMatrix;
}

/**
 * Multiply the current matrix by the perspective matrix
 * with the given clipping plane coordinates.
 */
void mglFrustum(MGLfloat left,
                MGLfloat right,
                MGLfloat bottom,
                MGLfloat top,
                MGLfloat near,
                MGLfloat far)
{
    mat4 proj_matrix;
    proj_matrix.make_zero();

    MGLfloat A = (right + left) / (right - left);
    MGLfloat B = (top + bottom) / (top - bottom);
    MGLfloat C = (far + near) / (far - near);
    MGLfloat D = (2 * far * near) / (far - near);

    proj_matrix(0, 2) = A;
    proj_matrix(1, 2) = B;
    proj_matrix(2, 2) = C;
    proj_matrix(2, 3) = D;

    proj_matrix(0, 0) = (2 * near) / (right - left);
    proj_matrix(1, 1) = (2 * near) / (top - bottom);
    proj_matrix(3, 2) = -1;

    mat4* currentMatrix = getCurrentMatrix();
    *currentMatrix = *currentMatrix * proj_matrix;
}

/**
 * Multiply the current matrix by the orthographic matrix
 * with the given clipping plane coordinates.
 */
void mglOrtho(MGLfloat left,
              MGLfloat right,
              MGLfloat bottom,
              MGLfloat top,
              MGLfloat near,
              MGLfloat far)
{
    mat4 ortho_matrix;
    ortho_matrix.make_zero();
    ortho_matrix(0, 0) = 2 / (right - left);
    ortho_matrix(1, 1) = 2 / (top - bottom);
    ortho_matrix(2, 2) = -2 / (far - near);
    ortho_matrix(3, 3) = 1;

    auto t_x =  (right + left)  / (left - right);
    auto t_y = (top + bottom) / (bottom - top);
    auto t_z = (far + near) / (near - far);
    ortho_matrix(0, 3) = t_x;
    ortho_matrix(1, 3) = t_y;
    ortho_matrix(2, 3) = t_z;
    
    mat4* currentMatrix = getCurrentMatrix();
    *currentMatrix = *currentMatrix * ortho_matrix;

    near_plane = near;
    far_plane = far;
}

/*
 * Set the current color for drawn shapes.
 */
void mglColor(MGLfloat red,
              MGLfloat green,
              MGLfloat blue)
{
    active_color[0] = red;
    active_color[1] = green;
    active_color[2] = blue;
}
