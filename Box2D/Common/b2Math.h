/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be pb2Minlainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#pragma once
#include <Box2D/Common/b2Settings.h>
#include <math.h>
#include <amp_math.h>

/// This function is used to ensure that a floating point number is not a NaN or infinity.
inline bool b2IsValid(float32 x)
{
	union {
		float32 f;
		int32 i;
	} v = { x };
	return (v.i & 0x7f800000) != 0x7f800000;
}
inline bool b2IsValid(float32 x) restrict(amp)
{
	union {
		float32 f;
		int32 i;
	} v = { x };
	return (v.i & 0x7f800000) != 0x7f800000;
}

/// This is a approximate yet fast inverse square-root.
inline float32 b2InvSqrt(float32 x)
{
	union
	{
		float32 x;
		int32 i;
	} convert;

	convert.x = x;
	float32 xhalf = 0.5f * x;
	convert.i = 0x5f3759df - (convert.i >> 1);
	x = convert.x;
	x = x * (1.5f - xhalf * x * x);
	return x;
}
inline float32 b2InvSqrt(float32 x) restrict(amp)
{
	union
	{
		float32 x;
		int32 i;
	} convert;

	convert.x = x;
	float32 xhalf = 0.5f * x;
	convert.i = 0x5f3759df - (convert.i >> 1);
	x = convert.x;
	x = x * (1.5f - xhalf * x * x);
	return x;
}
inline float32 b2InvCurt(float32 x) restrict(amp)
{
	union
	{
		float32 f;
		int32 i;
	} convert;

	convert.f = x;
	convert.i = 0x54a2fa8c - (convert.i / 3);
	float32 y = convert.f;
	y = (2.0f / 3) * y + 1 / (3.0f * y * y * x);
	y = (2.0f / 3) * y + 1 / (3.0f * y * y * x);
	return y;
}

#define ampMath concurrency::fast_math
#define	b2Sqrt(x)		sqrtf(x)
#define	ampSqrt(x)		ampMath::sqrtf(x)
#define	b2Atan2(y, x)	atan2f(y, x)
#define	ampAtan2(y, x)	ampMath::atan2f(y, x)
#define	ampSin(x)		ampMath::sinf(x)
#define	ampCos(x)		ampMath::cosf(x)
#define	ampFloor(x)		ampMath::floorf(x)
#define ampPow(x, p)	ampMath::powf(x, p)

struct b2Int2
{
	b2Int2() : y(0), x(0) {}
	b2Int2() restrict(amp) : y(0), x(0) {}

	int32 x, y;
};

/// A 2D column vector.
struct Vec2
{
	/// Default constructor does nothing (for performance).
	Vec2() : x(0), y(0) {}
	Vec2() restrict(amp) : x(0), y(0) {}

	/// Construct using coordinates
	Vec2(float32 v) : x(v), y(v) {}
	Vec2(float32 x, float32 y) : x(x), y(y) {}
	Vec2(float32 x, float32 y) restrict(amp) : x(x), y(y) {}

	/// Set this vector to all zeros.
	void SetZero() { x = 0.0f; y = 0.0f; }
	void SetZero() restrict(amp) { x = 0.0f; y = 0.0f; }

	/// Set this vector to some specified coordinates.
	void Set(float32 x_, float32 y_) { x = x_; y = y_; }
	void Set(float32 x_, float32 y_) restrict(amp) { x = x_; y = y_; }

	/// Negate this vector.
	Vec2 operator -() const { Vec2 v; v.Set(-x, -y); return v; }
	Vec2 operator -() const restrict(amp) { Vec2 v; v.Set(-x, -y); return v; }

	/// Read from and indexed element.
	float32 operator () (int32 i) const { return (&x)[i]; }
	float32 operator () (int32 i) const restrict(amp) { return (&x)[i]; }

	/// Write to an indexed element.
	float32& operator () (int32 i) { return (&x)[i]; }
	float32& operator () (int32 i) restrict(amp) { return (&x)[i]; }

	/// Add a vector to this vector.
	void operator += (const Vec2& v) { x += v.x; y += v.y; }
	void operator += (const Vec2& v) restrict(amp) { x += v.x; y += v.y; }

	/// Subtract a vector from this vector.
	void operator -= (const Vec2& v) { x -= v.x; y -= v.y; }
	void operator -= (const Vec2& v) restrict(amp) { x -= v.x; y -= v.y; }

	/// Multiply this vector by a scalar.
	void operator *= (float32 a) { x *= a; y *= a; }
	void operator *= (float32 a) restrict(amp) { x *= a; y *= a; }

	/// Get the length of this vector (the norm).
	float32 Length() const { return b2Sqrt(x * x + y * y); }
	float32 Length() const restrict(amp) { return ampSqrt(x * x + y * y); }

	/// Get the length squared. For performance, use this instead of
	/// Vec2::Length (if possible).
	float32 LengthSquared() const { return x * x + y * y; }
	float32 LengthSquared() const restrict(amp) { return x * x + y * y; }

	/// Convert this vector into a unit vector. Returns the length.
	float32 Normalize()
	{
		float32 length = Length();
		if (length < b2_epsilon) return 0.0f;
		float32 invLength = 1.0f / length;
		x *= invLength;
		y *= invLength;
		return length;
	}
	float32 Normalize() restrict(amp)
	{
		float32 length = Length();
		if (length < b2_epsilon) return 0.0f;
		float32 invLength = 1.0f / length;
		x *= invLength;
		y *= invLength;
		return length;
	}
	Vec2 Normalized() const restrict(amp)
	{
		float32 length = Length();
		if (length < b2_epsilon) return Vec2(0, 0);
		float32 invLength = 1.0f / length;
		return Vec2(x * invLength, y * invLength);
	}

	/// Does this vector contain finite coordinates?
	bool IsValid() const { return b2IsValid(x) && b2IsValid(y); }
	bool IsValid() const restrict(amp) { return b2IsValid(x) && b2IsValid(y); }

	/// Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
	Vec2 Skew() const { return Vec2(-y, x); }
	Vec2 Skew() const restrict(amp) { return Vec2(-y, x); }

	float32 x, y;
};

/// Add a float to a vector
inline Vec2 operator + (const Vec2& v, float f) { return Vec2(v.x + f, v.y + f); }
inline Vec2 operator + (const Vec2& v, float f) restrict(amp) { return Vec2(v.x + f, v.y + f); }

/// Substract a float from a vector.
inline Vec2 operator - (const Vec2& v, float f) { return Vec2(v.x - f, v.y - f); }
inline Vec2 operator - (const Vec2& v, float f) restrict(amp) { return Vec2(v.x - f, v.y - f); }

/// Multiply a float with a vector.
inline Vec2 operator * (const Vec2& v, float f) { return Vec2(v.x * f, v.y * f); }
inline Vec2 operator * (const Vec2& v, float f) restrict(amp) { return Vec2(v.x * f, v.y * f); }

/// Divide a vector by a float.
inline Vec2 operator / (const Vec2& v, float f) { return Vec2(v.x / f, v.y / f); }
inline Vec2 operator / (const Vec2& v, float f) restrict(amp) { return Vec2(v.x / f, v.y / f); }

/// A 3D column vector with 3 elements.
struct Vec3
{
	/// Default constructor does nothing (for performance).
	Vec3(): x(0), y(0), z(0) {}
	Vec3() restrict(amp) : x(0), y(0), z(0) {}

	/// Construct using coordinates.
	Vec3(float32 v) : x(v), y(v), z(v) {}
	Vec3(float32 x, float32 y, float32 z) : x(x), y(y), z(z) {}
	Vec3(float32 x, float32 y, float32 z) restrict(amp) : x(x), y(y), z(z) {}

	/// Construct using Vec2 und float.
	Vec3(Vec2 v, float32 z) : x(v.x), y(v.y), z(z) {}
	Vec3(Vec2 v, float32 z) restrict(amp) : x(v.x), y(v.y), z(z) {}

	/// Construct using Vec2 und float.
	Vec3(Vec2 v) : x(v.x), y(v.y), z(0) {}
	//Vec3(Vec2 v) restrict(amp) : x(v.x), y(v.y), z(0) {}

	/// Set this vector to all zeros.
	void SetZero() { x = 0.0f; y = 0.0f; z = 0.0f; }
	void SetZero() restrict(amp) { x = 0.0f; y = 0.0f; z = 0.0f; }

	/// Set this vector to some specified coordinates.
	void Set(float32 x_, float32 y_, float32 z_) { x = x_; y = y_; z = z_; }
	void Set(float32 x_, float32 y_, float32 z_) restrict(amp) { x = x_; y = y_; z = z_; }

	/// Negate this vector.
	Vec3 operator -() const { Vec3 v; v.Set(-x, -y, -z); return v; }
	Vec3 operator -() const restrict(amp) { Vec3 v; v.Set(-x, -y, -z); return v; }

	/// Add a vector to this vector.
	void operator += (const Vec3& v) { x += v.x; y += v.y; z += v.z; }
	void operator += (const Vec3& v) restrict(amp) { x += v.x; y += v.y; z += v.z; }
	void operator += (const Vec2& v) { x += v.x; y += v.y; }
	void operator += (const Vec2& v) restrict(amp) { x += v.x; y += v.y; }

	/// Subtract a vector from this vector.
	void operator -= (const Vec3& v) { x -= v.x; y -= v.y; z -= v.z; }
	void operator -= (const Vec3& v)  restrict(amp) { x -= v.x; y -= v.y; z -= v.z; }
	void operator -= (const Vec2& v) { x -= v.x; y -= v.y; }
	void operator -= (const Vec2& v)  restrict(amp) { x -= v.x; y -= v.y; }

	/// Multiply this vector by a scalar.
	void operator *= (float32 s) { x *= s; y *= s; z *= s; }
	void operator *= (float32 s) restrict(amp) { x *= s; y *= s; z *= s; }

	/// Multiply this vector by a scalar.
	void operator /= (float32 s) { x /= s; y /= s; z /= s; }
	void operator /= (float32 s) restrict(amp) { x /= s; y /= s; z /= s; }

	/// Get the length of this vector (the norm).
	float32 Length() const { return b2Sqrt(x * x + y * y + z * z); }
	float32 Length() const restrict(amp) { return ampSqrt(x * x + y * y + z * z); }

	/// Convert this vector into a unit vector. Returns the length.
	float32 Normalize()
	{
		float32 length = Length();
		if (length < b2_epsilon) return 0.0f;
		float32 invLength = 1.0f / length;
		x *= invLength;
		y *= invLength;
		z *= invLength;
		return length;
	}
	float32 Normalize() restrict(amp)
	{
		float32 length = Length();
		if (length < b2_epsilon) return 0.0f;
		float32 invLength = 1.0f / length;
		x *= invLength;
		y *= invLength;
		z *= invLength;
		return length;
	}
	Vec3 Normalized() const restrict(amp)
	{
		float32 length = Length();
		if (length < b2_epsilon) return Vec3(0, 0, 0);
		float32 invLength = 1.0f / length;
		return Vec3(x * invLength, y * invLength, z * invLength);
	}

	/// conversion to Vec2 (type-cast operator)
	operator Vec2() { return Vec2(x, y); }
	operator Vec2() restrict(amp) { return Vec2(x, y); }
	operator Vec2() const { return Vec2(x, y); }
	operator Vec2() const restrict(amp) { return Vec2(x, y); }

	float32 x, y, z;
};

/// A 4D column vector with 4 elements.
struct b2Vec4
{
	/// Default constructor does nothing (for performance).
	b2Vec4(): x(0), y(0), z(0), w(0) {}
	b2Vec4() restrict(amp) : x(0), y(0), z(0), w(0) {}

	/// Construct using coordinates.
	b2Vec4(float32 x, float32 y, float32 z, float32 w) : x(x), y(y), z(z), w(w) {}
	b2Vec4(float32 x, float32 y, float32 z, float32 w) restrict(amp) : x(x), y(y), z(z), w(w) {}

	float32 x, y, z, w;
};

/// A 2-by-2 matrix. Stored in column-major order.
struct b2Mat22
{
	/// The default constructor does nothing (for performance).
	b2Mat22() {}
	b2Mat22() restrict(amp) {}

	/// Construct this matrix using columns.
	b2Mat22(const Vec2& c1, const Vec2& c2) { ex = c1; ey = c2; }
	b2Mat22(const Vec2& c1, const Vec2& c2) restrict(amp) { ex = c1; ey = c2; }

	/// Construct this matrix using scalars.
	b2Mat22(float32 a11, float32 a12, float32 a21, float32 a22) { ex.x = a11; ex.y = a21; ey.x = a12; ey.y = a22; }
	b2Mat22(float32 a11, float32 a12, float32 a21, float32 a22) restrict(amp) { ex.x = a11; ex.y = a21; ey.x = a12; ey.y = a22; }

	/// Initialize this matrix using columns.
	void Set(const Vec2& c1, const Vec2& c2) { ex = c1; ey = c2; }
	void Set(const Vec2& c1, const Vec2& c2) restrict(amp) { ex = c1; ey = c2; }

	/// Set this to the identity matrix.
	void SetIdentity() { ex.x = 1.0f; ey.x = 0.0f; ex.y = 0.0f; ey.y = 1.0f; }
	void SetIdentity() restrict(amp) { ex.x = 1.0f; ey.x = 0.0f; ex.y = 0.0f; ey.y = 1.0f; }

	/// Set this matrix to all zeros.
	void SetZero() { ex.x = 0.0f; ey.x = 0.0f; ex.y = 0.0f; ey.y = 0.0f; }
	void SetZero() restrict(amp) { ex.x = 0.0f; ey.x = 0.0f; ex.y = 0.0f; ey.y = 0.0f; }

	b2Mat22 GetInverse() const
	{
		float32 a = ex.x, b = ey.x, c = ex.y, d = ey.y;
		b2Mat22 B;
		float32 det = a * d - b * c;
		if (det != 0.0f) det = 1.0f / det;
		B.ex.x = det * d;	B.ey.x = -det * b;
		B.ex.y = -det * c;	B.ey.y = det * a;
		return B;
	}
	b2Mat22 GetInverse() const restrict(amp)
	{
		float32 a = ex.x, b = ey.x, c = ex.y, d = ey.y;
		b2Mat22 B;
		float32 det = a * d - b * c;
		if (det != 0.0f) det = 1.0f / det;
		B.ex.x = det * d;	B.ey.x = -det * b;
		B.ex.y = -det * c;	B.ey.y = det * a;
		return B;
	}

	/// Solve A * x = b, where b is a column vector. This is more efficient
	/// than computing the inverse in one-shot cases.
	Vec2 Solve(const Vec2& b) const
	{
		float32 a11 = ex.x, a12 = ey.x, a21 = ex.y, a22 = ey.y;
		float32 det = a11 * a22 - a12 * a21;
		if (det != 0.0f) det = 1.0f / det;
		Vec2 x;
		x.x = det * (a22 * b.x - a12 * b.y);
		x.y = det * (a11 * b.y - a21 * b.x);
		return x;
	}
	Vec2 Solve(const Vec2& b) const restrict(amp)
	{
		float32 a11 = ex.x, a12 = ey.x, a21 = ex.y, a22 = ey.y;
		float32 det = a11 * a22 - a12 * a21;
		if (det != 0.0f) det = 1.0f / det;
		Vec2 x;
		x.x = det * (a22 * b.x - a12 * b.y);
		x.y = det * (a11 * b.y - a21 * b.x);
		return x;
	}

	Vec2 ex, ey;
};

/// A 3-by-3 matrix. Stored in column-major order.
struct b2Mat33
{
	/// The default constructor does nothing (for performance).
	b2Mat33() {}
	b2Mat33() restrict(amp) {}

	/// Construct this matrix using columns.
	b2Mat33(const Vec3& c1, const Vec3& c2, const Vec3& c3) { ex = c1; ey = c2; ez = c3; }
	b2Mat33(const Vec3& c1, const Vec3& c2, const Vec3& c3) restrict(amp) { ex = c1; ey = c2; ez = c3; }

	/// Set this matrix to all zeros.
	void SetZero() { ex.SetZero(); ey.SetZero(); ez.SetZero(); }
	void SetZero() restrict(amp) { ex.SetZero(); ey.SetZero(); ez.SetZero(); }

	/// Solve A * x = b, where b is a column vector. This is more efficient
	/// than computing the inverse in one-shot cases.
	Vec3 Solve33(const Vec3& b) const;
	Vec3 Solve33(const Vec3& b) const restrict(amp);

	/// Solve A * x = b, where b is a column vector. This is more efficient
	/// than computing the inverse in one-shot cases. Solve only the upper
	/// 2-by-2 matrix equation.
	Vec2 Solve22(const Vec2& b) const;
	Vec2 Solve22(const Vec2& b) const restrict(amp);

	/// Get the inverse of this matrix as a 2-by-2.
	/// Returns the zero matrix if singular.
	void GetInverse22(b2Mat33* M) const;
	void GetInverse22(b2Mat33* M) const restrict(amp);

	/// Get the symmetric inverse of this matrix as a 3-by-3.
	/// Returns the zero matrix if singular.
	void GetSymInverse33(b2Mat33* M) const;
	void GetSymInverse33(b2Mat33* M) const restrict(amp);

	Vec3 ex, ey, ez;
};

/// Rotation
struct b2Rot
{
	b2Rot(): s(0), c(0) {}
	b2Rot() restrict(amp) : s(0), c(0) {}

	/// Initialize from an angle in radians
	explicit b2Rot(float32 angle) { s = sinf(angle); c = cosf(angle); }
	explicit b2Rot(float32 angle) restrict(amp) { s = ampMath::sinf(angle); c = ampMath::cosf(angle); }

	/// Set using an angle in radians.
	void Set(float32 angle) { s = sinf(angle); c = cosf(angle); }
	void Set(float32 angle) restrict(amp) { s = ampSin(angle); c = ampCos(angle); }

	/// Set to the identity rotation
	void SetIdentity() { s = 0.0f; c = 1.0f; }
	void SetIdentity() restrict(amp) { s = 0.0f; c = 1.0f; }

	/// Get the angle in radians
	float32 GetAngle() const { return b2Atan2(s, c); }
	float32 GetAngle() const restrict(amp) { return ampAtan2(s, c); }

	/// Get the x-axis
	Vec2 GetXAxis() const { return Vec2(c, s); }
	Vec2 GetXAxis() const restrict(amp) { return Vec2(c, s); }

	/// Get the u-axis
	Vec2 GetYAxis() const { return Vec2(-s, c); }
	Vec2 GetYAxis() const restrict(amp) { return Vec2(-s, c); }

	/// Sine and cosine
	float32 s, c;
};

/// A transform contains translation and rotation. It is used to represent
/// the position and orientation of rigid frames.
struct b2Transform
{
	/// The default constructor does nothing.
	b2Transform() { p.SetZero(); q.SetIdentity(); z = 0; }
	b2Transform() restrict(amp) { p.SetZero(); q.SetIdentity(); z = 0; }

	/// Initialize using a position vector and a rotation.
	b2Transform(const Vec2& position, const b2Rot& rotation) : p(position), q(rotation) {}
	b2Transform(const Vec2& position, const b2Rot& rotation) restrict(amp) : p(position), q(rotation) {}
	b2Transform(const Vec3& position, const b2Rot& rotation) : p(position), z(position.z), q(rotation) {}
	b2Transform(const Vec3& position, const b2Rot& rotation) restrict(amp) : p(position), z(position.z), q(rotation) {}
	b2Transform(float32 posX, float32 posY, const b2Rot& rotation) : p(Vec2(posX, posY)), q(rotation) {}
	b2Transform(float32 posX, float32 posY, const b2Rot& rotation) restrict(amp) : p(Vec2(posX, posY)), q(rotation) {}

	/// Set this to the identity transform.
	void SetIdentity() { p.SetZero(); q.SetIdentity();  z = 0; }
	void SetIdentity() restrict(amp) { p.SetZero(); q.SetIdentity(); z = 0; }

	/// Set this based on the position and angle.
	void Set(const Vec2& position, float32 angle) { p = position; q.Set(angle); }
	void Set(const Vec2& position, float32 angle) restrict(amp) { p = position; q.Set(angle); }

#if LIQUIDFUN_EXTERNAL_LANGUAGE_API
	/// Get x-coordinate of p.
	float32 GetPositionX() const { return p.x; }

	/// Get y-coordinate of p.
	float32 GetPositionY() const { return p.y; }

	/// Get sine-component of q.
	float32 GetRotationSin() const { return q.s; }

	/// Get cosine-component of q.
	float32 GetRotationCos() const { return q.c; }
#endif // LIQUIDFUN_EXTERNAL_LANGUAGE_API

	Vec2 p;
	float32 z;
	b2Rot q;
};

/// This describes the motion of a body/shape for TOI computation.
/// Shapes are defined with respect to the body origin, which may
/// no coincide with the center of mass. However, to support dynamics
/// we must interpolate the center of mass position.
struct b2Sweep
{
	/// Get the interpolated transform at a specific time.
	/// @param beta is a factor in [0,1], where 0 indicates alpha0.
	void GetTransform(b2Transform& xfb, float32 beta) const;
	void GetTransform(b2Transform& xfb, float32 beta) const restrict(amp);

	/// Advance the sweep forward, yielding a new initial state.
	/// @param alpha the new initial time.
	void Advance(float32 alpha);
	void Advance(float32 alpha) restrict(amp);

	/// Normalize the angles.
	void Normalize();
	void Normalize() restrict(amp);

	Vec2 localCenter;	///< local center of mass position
	Vec2 c0, c;		///< center world positions
	float32 a0, a;		///< world angles

	/// Fraction of the current time step in the range [0,1]
	/// c0 and a0 are the positions at alpha0.
	float32 alpha0;
};

/// Useful constant
extern const Vec2 Vec2_zero;
extern const Vec3 Vec3_zero;
extern const Vec3 Vec3_up;

/// Perform the dot product on two vectors.
inline float32 b2Dot(const Vec2& a, const Vec2& b) { return a.x * b.x + a.y * b.y; }
inline float32 b2Dot(const Vec2& a, const Vec2& b) restrict(amp) { return a.x * b.x + a.y * b.y; }
inline float32 b2Dot(const float32& ax, const float32& ay, const float32& bx, const float32& by) { return ax * bx + ay * by; }
inline float32 b2Dot(const float32& ax, const float32& ay, const float32& bx, const float32& by) restrict(amp) { return ax * bx + ay * by; }
inline void Normalize(float32& x, float32& y) { float32 length = b2Sqrt(x * x + y * y); x /= length; y /= length; }
inline void Normalize(float32& x, float32& y) restrict(amp) { float32 length = ampSqrt(x * x + y * y); x /= length; y /= length; }
inline void Normalize(float32& x, float32& y, float32& z) { float32 length = b2Sqrt(x * x + y * y + z * z); x /= length; y /= length; z /= length; }
inline void Normalize(float32& x, float32& y, float32& z) restrict(amp) { float32 length = ampSqrt(x * x + y * y + z * z); x /= length; y /= length; z /= length; }

/// Perform the cross product on two vectors. In 2D this produces a scalar.
inline float32 b2Cross(const Vec2& a, const Vec2& b) { return a.x* b.y - a.y * b.x; }
inline float32 b2Cross(const Vec2& a, const Vec2& b) restrict(amp) { return a.x* b.y - a.y * b.x; }

inline float32 b2Cross(const float32& ax, const float32& ay, const float32& bx, const float32& by) { return ax * by - ay * bx; }
inline float32 b2Cross(const float32& ax, const float32& ay, const float32& bx, const float32& by) restrict(amp) { return ax * by - ay * bx; }
inline float32 b2Cross2D(const Vec3& a, const Vec3& b) { return a.x* b.y - a.y * b.x; }
inline float32 b2Cross2D(const Vec3& a, const Vec3& b) restrict(amp) { return a.x* b.y - a.y * b.x; }

/// Perform the cross product on a vector and a scalar. In 2D this produces
/// a vector.
inline Vec2 b2Cross(const Vec2& a, float32 s) { return Vec2(s * a.y, -s * a.x); }
inline Vec2 b2Cross(const Vec2& a, float32 s) restrict(amp) { return Vec2(s * a.y, -s * a.x); }

/// Perform the cross product on a scalar and a vector. In 2D this produces
/// a vector.
inline Vec2 b2Cross(float32 s, const Vec2& a) { return Vec2(-s * a.y, s * a.x); }
inline Vec2 b2Cross(float32 s, const Vec2& a) restrict(amp) { return Vec2(-s * a.y, s * a.x); }

/// Multiply a matrix times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another.
inline Vec2 b2Mul(const b2Mat22& A, const Vec2& v) { return Vec2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y); }
inline Vec2 b2Mul(const b2Mat22& A, const Vec2& v) restrict(amp) { return Vec2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y); }

/// Multiply a matrix transpose times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another (inverse transform).
inline Vec2 b2MulT(const b2Mat22& A, const Vec2& v) { return Vec2(b2Dot(v, A.ex), b2Dot(v, A.ey)); }
inline Vec2 b2MulT(const b2Mat22& A, const Vec2& v) restrict(amp) { return Vec2(b2Dot(v, A.ex), b2Dot(v, A.ey)); }

/// Add two vectors component-wise.
inline Vec2 operator + (const Vec2& a, const Vec2& b) { return Vec2(a.x + b.x, a.y + b.y); }
inline Vec2 operator + (const Vec2& a, const Vec2& b) restrict(amp) { return Vec2(a.x + b.x, a.y + b.y); }

/// Subtract two vectors component-wise.
inline Vec2 operator - (const Vec2& a, const Vec2& b) { return Vec2(a.x - b.x, a.y - b.y); }
inline Vec2 operator - (const Vec2& a, const Vec2& b) restrict(amp) { return Vec2(a.x - b.x, a.y - b.y); }

inline Vec2 operator * (float32 s, const Vec2& a) { return Vec2(s * a.x, s * a.y); }
inline Vec2 operator * (float32 s, const Vec2& a) restrict(amp) { return Vec2(s * a.x, s * a.y); }

inline bool operator == (const Vec2& a, const Vec2& b) { return a.x == b.x && a.y == b.y; }
inline bool operator == (const Vec2& a, const Vec2& b) restrict(amp) { return a.x == b.x && a.y == b.y; }

inline bool operator != (const Vec2& a, const Vec2& b) { return !operator==(a, b); }
inline bool operator != (const Vec2& a, const Vec2& b) restrict(amp) { return !operator==(a, b); }

inline float32 b2Distance(const Vec2& a, const Vec2& b) { Vec2 c = a - b; return c.Length(); }
inline float32 b2Distance(const Vec2& a, const Vec2& b) restrict(amp) { Vec2 c = a - b; return c.Length(); }

inline float32 b2DistanceSquared(const Vec2& a, const Vec2& b) { Vec2 c = a - b; return b2Dot(c, c); }
inline float32 b2DistanceSquared(const Vec2& a, const Vec2& b) restrict(amp) { Vec2 c = a - b; return b2Dot(c, c); }

inline Vec3 operator * (float32 s, const Vec3& a) { return Vec3(s * a.x, s * a.y, s * a.z); }
inline Vec3 operator * (float32 s, const Vec3& a) restrict(amp) { return Vec3(s * a.x, s * a.y, s * a.z); }
inline Vec3 operator * (const Vec3& a, float32 s) { return Vec3(s * a.x, s * a.y, s * a.z); }
inline Vec3 operator * (const Vec3& a, float32 s) restrict(amp) { return Vec3(s * a.x, s * a.y, s * a.z); }
inline float32 operator * (const Vec3& a, const Vec3& b) { return a.x* b.x + a.y * b.y + a.z * b.z; }
inline float32 operator * (const Vec3& a, const Vec3& b) restrict(amp) { return a.x* b.x + a.y * b.y + a.z * b.z; }
inline Vec3 operator / (const Vec3& a, float32 s) { return Vec3(a.x / s, a.y / s, a.z / s); }
inline Vec3 operator / (const Vec3& a, float32 s) restrict(amp) { return Vec3(a.x / s, a.y / s, a.z / s); }

/// Add two vectors component-wise.
inline Vec3 operator + (const Vec3& a, const Vec3& b) { return Vec3(a.x + b.x, a.y + b.y, a.z + b.z); }
inline Vec3 operator + (const Vec3& a, const Vec3& b) restrict(amp) { return Vec3(a.x + b.x, a.y + b.y, a.z + b.z); }
inline Vec3 operator + (const Vec3& a, const Vec2& b) { return Vec3(a.x + b.x, a.y + b.y, a.z); }
inline Vec3 operator + (const Vec3& a, const Vec2& b) restrict(amp) { return Vec3(a.x + b.x, a.y + b.y, a.z); }
inline Vec2 operator + (const Vec2& a, const Vec3& b) { return Vec2(a.x + b.x, a.y + b.y); }
inline Vec2 operator + (const Vec2& a, const Vec3& b) restrict(amp) { return Vec2(a.x + b.x, a.y + b.y); }

/// Subtract two vectors component-wise.
inline Vec3 operator - (const Vec3& a, const Vec3& b) { return Vec3(a.x - b.x, a.y - b.y, a.z - b.z); }
inline Vec3 operator - (const Vec3& a, const Vec3& b) restrict(amp) { return Vec3(a.x - b.x, a.y - b.y, a.z - b.z); }
inline Vec3 operator - (const Vec3& a, const Vec2& b) { return Vec3(a.x - b.x, a.y - b.y, a.z); }
inline Vec3 operator - (const Vec3& a, const Vec2& b) restrict(amp) { return Vec3(a.x - b.x, a.y - b.y, a.z); }
inline Vec2 operator - (const Vec2& a, const Vec3& b) { return Vec2(a.x - b.x, a.y - b.y); }
inline Vec2 operator - (const Vec2& a, const Vec3& b) restrict(amp) { return Vec2(a.x - b.x, a.y - b.y); }

inline bool operator < (const Vec3& a, const Vec3& b) { return a.x < b.x && a.y < b.y && a.z < b.z; }
inline bool operator < (const Vec3& a, const Vec3& b) restrict(amp) { return a.x < b.x && a.y < b.y && a.z < b.z; }
inline bool operator > (const Vec3& a, const Vec3& b) { return a.x > b.x && a.y > b.y && a.z > b.z; }
inline bool operator > (const Vec3& a, const Vec3& b) restrict(amp) { return a.x > b.x && a.y > b.y && a.z > b.z; }

inline bool operator < (const Vec3& a, const float32& b) { return a.x < b && a.y < b && a.z < b; }
inline bool operator < (const Vec3& a, const float32& b) restrict(amp) { return a.x < b && a.y < b && a.z < b; }
inline bool operator > (const Vec3& a, const float32& b) { return a.x > b && a.y > b && a.z > b; }
inline bool operator > (const Vec3& a, const float32& b) restrict(amp) { return a.x > b && a.y > b && a.z > b; }

inline float32 b2Distance(const Vec3& a, const Vec3& b) { Vec3 c = a - b; return c.Length(); }
inline float32 b2Distance(const Vec3& a, const Vec3& b) restrict(amp) { Vec3 c = a - b; return c.Length(); }

/// Perform the dot product on two vectors.
inline float32 b2Dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline float32 b2Dot(const Vec3& a, const Vec3& b) restrict(amp) { return a.x * b.x + a.y * b.y + a.z * b.z; }

/// Perform the cross product on two vectors.
inline Vec3 b2Cross(const Vec3& a, const Vec3& b) { return Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }
inline Vec3 b2Cross(const Vec3& a, const Vec3& b) restrict(amp) { return Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }

inline b2Mat22 operator + (const b2Mat22& A, const b2Mat22& B) { return b2Mat22(A.ex + B.ex, A.ey + B.ey); }
inline b2Mat22 operator + (const b2Mat22& A, const b2Mat22& B) restrict(amp) { return b2Mat22(A.ex + B.ex, A.ey + B.ey); }

// A * B
inline b2Mat22 b2Mul(const b2Mat22& A, const b2Mat22& B) { return b2Mat22(b2Mul(A, B.ex), b2Mul(A, B.ey)); }
inline b2Mat22 b2Mul(const b2Mat22& A, const b2Mat22& B) restrict(amp) { return b2Mat22(b2Mul(A, B.ex), b2Mul(A, B.ey)); }

// A^T * B
inline b2Mat22 b2MulT(const b2Mat22& A, const b2Mat22& B) { Vec2 c1(b2Dot(A.ex, B.ex), b2Dot(A.ey, B.ex)); Vec2 c2(b2Dot(A.ex, B.ey), b2Dot(A.ey, B.ey)); return b2Mat22(c1, c2); }
inline b2Mat22 b2MulT(const b2Mat22& A, const b2Mat22& B) restrict(amp) { Vec2 c1(b2Dot(A.ex, B.ex), b2Dot(A.ey, B.ex)); Vec2 c2(b2Dot(A.ex, B.ey), b2Dot(A.ey, B.ey)); return b2Mat22(c1, c2); }

/// Multiply a matrix times a vector.
inline Vec3 b2Mul(const b2Mat33 & A, const Vec3& v) { return v.x* A.ex + v.y * A.ey + v.z * A.ez; }
inline Vec3 b2Mul(const b2Mat33 & A, const Vec3& v) restrict(amp) { return v.x* A.ex + v.y * A.ey + v.z * A.ez; }

/// Multiply a matrix times a vector.
inline Vec2 b2Mul22(const b2Mat33 & A, const Vec2& v) { return Vec2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y); }
inline Vec2 b2Mul22(const b2Mat33 & A, const Vec2& v) restrict(amp) { return Vec2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y); }

/// Multiply two rotations: q * r
inline b2Rot b2Mul(const b2Rot & q, const b2Rot & r) { b2Rot qr; qr.s = q.s * r.c + q.c * r.s; qr.c = q.c * r.c - q.s * r.s; return qr; }
inline b2Rot b2Mul(const b2Rot & q, const b2Rot & r) restrict(amp) { b2Rot qr; qr.s = q.s * r.c + q.c * r.s; qr.c = q.c * r.c - q.s * r.s; return qr; }

/// Transpose multiply two rotations: qT * r
inline b2Rot b2MulT(const b2Rot & q, const b2Rot & r) { b2Rot qr; qr.s = q.c * r.s - q.s * r.c; qr.c = q.c * r.c + q.s * r.s; return qr; }
inline b2Rot b2MulT(const b2Rot & q, const b2Rot & r) restrict(amp) { b2Rot qr; qr.s = q.c * r.s - q.s * r.c; qr.c = q.c * r.c + q.s * r.s; return qr; }

/// Rotate a vector
inline Vec2 b2Mul(const b2Rot& q, const Vec2& v) { return Vec2(q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y); }
inline Vec2 b2Mul(const b2Rot& q, const Vec2& v) restrict(amp) { return Vec2(q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y); }
inline Vec3 b2Mul(const b2Rot& q, const Vec3& v) { return Vec3(q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y, v.z); }
inline Vec3 b2Mul(const b2Rot& q, const Vec3& v) restrict(amp) { return Vec3(q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y, v.z); }
inline float32 b2MulX(const b2Rot & q, float32 x, float32 y) { return q.c* x - q.s * y; }
inline float32 b2MulX(const b2Rot & q, float32 x, float32 y) restrict(amp) { return q.c* x - q.s * y; }
inline float32 b2MulY(const b2Rot & q, float32 x, float32 y) { return q.s* x + q.c * y; }
inline float32 b2MulY(const b2Rot & q, float32 x, float32 y) restrict(amp) { return q.s* x + q.c * y; }

/// Inverse rotate a vector
inline Vec2 b2MulT(const b2Rot & q, const Vec2& v) { return Vec2(q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y); }
inline Vec2 b2MulT(const b2Rot & q, const Vec2& v) restrict(amp) { return Vec2(q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y); }

inline Vec2 b2Mul(const b2Transform & T, const Vec2& v) { float32 x = (T.q.c * v.x - T.q.s * v.y) + T.p.x; float32 y = (T.q.s * v.x + T.q.c * v.y) + T.p.y; return Vec2(x, y); }
inline Vec2 b2Mul(const b2Transform & T, const Vec2& v) restrict(amp) { float32 x = (T.q.c * v.x - T.q.s * v.y) + T.p.x; float32 y = (T.q.s * v.x + T.q.c * v.y) + T.p.y; return Vec2(x, y); }
inline Vec2 b2Mul(const b2Transform & T, const Vec3& v) { float32 x = (T.q.c * v.x - T.q.s * v.y) + T.p.x; float32 y = (T.q.s * v.x + T.q.c * v.y) + T.p.y; return Vec2(x, y); }
inline Vec2 b2Mul(const b2Transform& T, const Vec3& v) restrict(amp) { float32 x = (T.q.c * v.x - T.q.s * v.y) + T.p.x; float32 y = (T.q.s * v.x + T.q.c * v.y) + T.p.y; return Vec2(x, y); }
inline Vec3 b2Mul3D(const b2Transform& T, const Vec3& v) { return Vec3((T.q.c * v.x - T.q.s * v.y) + T.p.x, (T.q.s * v.x + T.q.c * v.y) + T.p.y, v.z + T.z); }
inline Vec3 b2Mul3D(const b2Transform & T, const Vec3& v) restrict(amp) { return Vec3((T.q.c * v.x - T.q.s * v.y) + T.p.x, (T.q.s * v.x + T.q.c * v.y) + T.p.y, v.z + T.z); }
inline float32 b2MulX(const b2Transform & T, float32 x, float32 y) { return (T.q.c * x - T.q.s * y) + T.p.x; }
inline float32 b2MulX(const b2Transform & T, float32 x, float32 y) restrict(amp) { return (T.q.c * x - T.q.s * y) + T.p.x; }
inline float32 b2MulY(const b2Transform & T, float32 x, float32 y) { return (T.q.s * x + T.q.c * y) + T.p.y; }
inline float32 b2MulY(const b2Transform & T, float32 x, float32 y) restrict(amp) { return (T.q.s * x + T.q.c * y) + T.p.y; }

inline Vec2 b2MulT(const b2Transform & T, const Vec2& v) { float32 px = v.x - T.p.x; float32 py = v.y - T.p.y; float32 x = (T.q.c * px + T.q.s * py); float32 y = (-T.q.s * px + T.q.c * py); return Vec2(x, y); }
inline Vec2 b2MulT(const b2Transform & T, const Vec2& v) restrict(amp) { float32 px = v.x - T.p.x; float32 py = v.y - T.p.y; float32 x = (T.q.c * px + T.q.s * py); float32 y = (-T.q.s * px + T.q.c * py); return Vec2(x, y); }

// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
//    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
inline b2Transform b2Mul(const b2Transform & A, const b2Transform & B) { b2Transform C; C.q = b2Mul(A.q, B.q); C.p = b2Mul(A.q, B.p) + A.p; return C; }
inline b2Transform b2Mul(const b2Transform & A, const b2Transform & B) restrict(amp) { b2Transform C; C.q = b2Mul(A.q, B.q); C.p = b2Mul(A.q, B.p) + A.p; return C; }

// v2 = A.q' * (B.q * v1 + B.p - A.p)
//    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
inline b2Transform b2MulT(const b2Transform & A, const b2Transform & B) { b2Transform C; C.q = b2MulT(A.q, B.q); C.p = b2MulT(A.q, B.p - A.p); return C; }
inline b2Transform b2MulT(const b2Transform & A, const b2Transform & B) restrict(amp) { b2Transform C; C.q = b2MulT(A.q, B.q); C.p = b2MulT(A.q, B.p - A.p); return C; }

template <typename T> inline T b2Abs(T a) { return a > T(0) ? a : -a; }
template <typename T> inline T b2Abs(T a) restrict(amp) { return a > T(0) ? a : -a; }
inline Vec3 b2Abs(Vec3 a) restrict(amp) { return Vec3(b2Abs(a.x), b2Abs(a.y), b2Abs(a.z)); }

inline Vec2 b2Abs(const Vec2& a) { return Vec2(b2Abs(a.x), b2Abs(a.y)); }
inline Vec2 b2Abs(const Vec2& a) restrict(amp) { return Vec2(b2Abs(a.x), b2Abs(a.y)); }

inline b2Mat22 b2Abs(const b2Mat22& A) { return b2Mat22(b2Abs(A.ex), b2Abs(A.ey)); }
inline b2Mat22 b2Abs(const b2Mat22& A) restrict(amp) { return b2Mat22(b2Abs(A.ex), b2Abs(A.ey)); }

template <typename T> inline T b2Min(T a, T b) { return a < b ? a : b; }
template <typename T> inline T b2Min(T a, T b) restrict(amp) { return a < b ? a : b; }
template <typename T> inline T b2AbsMin(T a, T b) { return b2Abs(a) < b2Abs(b) ? a : b; }
template <typename T> inline T b2AbsMin(T a, T b) restrict(amp) { return b2Abs(a) < b2Abs(b) ? a : b; }

inline Vec2 b2Min(const Vec2& a, const Vec2& b) { return Vec2(b2Min(a.x, b.x), b2Min(a.y, b.y)); }
inline Vec2 b2Min(const Vec2& a, const Vec2& b) restrict(amp) { return Vec2(b2Min(a.x, b.x), b2Min(a.y, b.y)); }

inline Vec3 b2Min(const Vec3& a, const Vec3& b) { return Vec3(b2Min(a.x, b.x), b2Min(a.y, b.y), b2Min(a.z, b.z)); }
inline Vec3 b2Min(const Vec3& a, const Vec3& b) restrict(amp) { return Vec3(b2Min(a.x, b.x), b2Min(a.y, b.y), b2Min(a.z, b.z)); }
inline Vec3 b2AbsMin(const Vec3& a, const Vec3& b) { return a.Length() < b.Length() ? a : b; }
inline Vec3 b2AbsMin(const Vec3& a, const Vec3& b) restrict(amp) { return a.Length() < b.Length() ? a : b; }

template <typename T> inline T b2Max(T a, T b) { return a > b ? a : b; }
template <typename T> inline T b2Max(T a, T b) restrict(amp) { return a > b ? a : b; }
template <typename T> inline T b2AbsMax(T a, T b) { return b2Abs(a) > b2Abs(b) ? a : b; }
template <typename T> inline T b2AbsMax(T a, T b) restrict(amp) { return b2Abs(a) > b2Abs(b) ? a : b; }

inline Vec2 b2Max(const Vec2& a, const Vec2& b) { return Vec2(b2Max(a.x, b.x), b2Max(a.y, b.y)); }
inline Vec2 b2Max(const Vec2& a, const Vec2& b) restrict(amp) { return Vec2(b2Max(a.x, b.x), b2Max(a.y, b.y)); }

template <typename T> inline T b2Clamp(T a, T low, T high) { return b2Max(low, b2Min(a, high)); }
template <typename T> inline T b2Clamp(T a, T low, T high) restrict(amp) { return b2Max(low, b2Min(a, high)); }

inline Vec2 b2Clamp(const Vec2& a, const Vec2& low, const Vec2& high) { return b2Max(low, b2Min(a, high)); }
inline Vec2 b2Clamp(const Vec2& a, const Vec2& low, const Vec2& high) restrict(amp) { return b2Max(low, b2Min(a, high)); }

template<typename T> inline void b2Swap(T & a, T & b) { T tmp = a; a = b; b = tmp; }
template<typename T> inline void b2Swap(T & a, T & b) restrict(amp) { T tmp = a; a = b; b = tmp; }

/// "Next Largest Power of 2
/// Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
/// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
/// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
/// largest power of 2. For a 32-bit value:"
inline uint32 b2NextPowerOfTwo(uint32 x) { x |= (x >> 1); x |= (x >> 2); x |= (x >> 4); x |= (x >> 8); x |= (x >> 16); return x + 1; }
inline uint32 b2NextPowerOfTwo(uint32 x) restrict(amp) { x |= (x >> 1); x |= (x >> 2); x |= (x >> 4); x |= (x >> 8); x |= (x >> 16); return x + 1; }

inline bool b2IsPowerOfTwo(uint32 x) { bool result = x > 0 && (x & (x - 1)) == 0; return result; }
inline bool b2IsPowerOfTwo(uint32 x) restrict(amp) { bool result = x > 0 && (x & (x - 1)) == 0; return result; }

inline void b2Sweep::GetTransform(b2Transform& xf, float32 beta) const
{
	xf.p = (1.0f - beta) * c0 + beta * c;
	float32 angle = (1.0f - beta) * a0 + beta * a;
	xf.q.Set(angle);
	xf.p -= b2Mul(xf.q, localCenter);
}
inline void b2Sweep::GetTransform(b2Transform& xf, float32 beta) const restrict(amp)
{
	xf.p = (1.0f - beta) * c0 + beta * c;
	float32 angle = (1.0f - beta) * a0 + beta * a;
	xf.q.Set(angle);
	xf.p -= b2Mul(xf.q, localCenter);
}

inline void b2Sweep::Advance(float32 alpha)
{
	float32 beta = (alpha - alpha0) / (1.0f - alpha0);
	c0 += beta * (c - c0);
	a0 += beta * (a - a0);
	alpha0 = alpha;
}
inline void b2Sweep::Advance(float32 alpha) restrict(amp)
{
	float32 beta = (alpha - alpha0) / (1.0f - alpha0);
	c0 += beta * (c - c0);
	a0 += beta * (a - a0);
	alpha0 = alpha;
}

/// Normalize an angle in radians to be between -pi and pi
inline void b2Sweep::Normalize()
{
	float32 twoPi = 2.0f * b2_pi;
	float32 d = twoPi * floorf(a0 / twoPi);
	a0 -= d;
	a -= d;
}
inline void b2Sweep::Normalize() restrict(amp)
{
	float32 twoPi = 2.0f * b2_pi;
	float32 d = twoPi * ampFloor(a0 / twoPi);
	a0 -= d;
	a -= d;
}

inline float32 Random(const float32 min = 0, const float32 max = 1)
{
	return min + (((float32)rand()) / (float32)RAND_MAX) * (max - min);
}

inline bool IsBetween(float32 x, float32 a, float32 b, float32 tol)
{
	return a < b ? x + tol >= a && a + x - tol <= b
				 : x + tol >= b && b + x - tol <= a;
}
inline bool IsBetween(float32 x, float32 a, float32 b, float32 tol) restrict(amp)
{
	return a < b ? x + tol >= a && x - tol <= b
		         : x + tol >= b && x - tol <= a;
}

struct Future
{
private:
	std::future<void> m_future;

public:
	template<typename F>
	void RunAsync(const F& func) { m_future = std::async(std::launch::async, func); }
	void wait() { if (m_future.valid()) m_future.wait(); }
};
