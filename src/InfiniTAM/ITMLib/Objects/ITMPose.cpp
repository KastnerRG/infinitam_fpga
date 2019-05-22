// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include <math.h>
#include "ITMPose.h"

#include <stdio.h>

using namespace ITMLib::Objects;

ITMPose::ITMPose(void) { this->SetFrom(0, 0, 0, 0, 0, 0); }

ITMPose::ITMPose(float tx, float ty, float tz, float rx, float ry, float rz) 
{ this->SetFrom(tx, ty, tz, rx, ry, rz); }
ITMPose::ITMPose(const float pose[6]) { this->SetFrom(pose); }
ITMPose::ITMPose(const Matrix4f & src) { this->SetM(src); }
ITMPose::ITMPose(const Vector6f & tangent) { this->SetFrom(tangent); }
ITMPose::ITMPose(const ITMPose & src) { this->SetFrom(&src); }

#ifndef M_SQRT1_2
#define M_SQRT1_2 0.707106781186547524401
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.5707963267948966192E0
#endif

void ITMPose::SetFrom(float tx, float ty, float tz, float rx, float ry, float rz)
{
	this->params.each.tx = tx;
	this->params.each.ty = ty;
	this->params.each.tz = tz;
	this->params.each.rx = rx;
	this->params.each.ry = ry;
	this->params.each.rz = rz;

	this->SetModelViewFromParams();
}

void ITMPose::SetFrom(const Vector3f &translation, const Vector3f &rotation)
{
	this->params.each.tx = translation.x;
	this->params.each.ty = translation.y;
	this->params.each.tz = translation.z;
	this->params.each.rx = rotation.x;
	this->params.each.ry = rotation.y;
	this->params.each.rz = rotation.z;

	this->SetModelViewFromParams();
}

void ITMPose::SetFrom(const Vector6f &tangent)
{
	this->params.each.tx = tangent[0];
	this->params.each.ty = tangent[1];
	this->params.each.tz = tangent[2];
	this->params.each.rx = tangent[3];
	this->params.each.ry = tangent[4];
	this->params.each.rz = tangent[5];

	this->SetModelViewFromParams();
}

void ITMPose::SetFrom(const float pose[6])
{
	SetFrom(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
}

void ITMPose::SetFrom(const ITMPose *pose)
{
	this->params.each.tx = pose->params.each.tx;
	this->params.each.ty = pose->params.each.ty;
	this->params.each.tz = pose->params.each.tz;
	this->params.each.rx = pose->params.each.rx;
	this->params.each.ry = pose->params.each.ry;
	this->params.each.rz = pose->params.each.rz;

	M = pose->M;
}

void ITMPose::SetModelViewFromParams()
{
	float one_6th = 1.0f/6.0f;
	float one_20th = 1.0f/20.0f;

	Vector3f w; w.x = params.each.rx; w.y = params.each.ry; w.z = params.each.rz;
	Vector3f t; t.x = params.each.tx; t.y = params.each.ty; t.z = params.each.tz;

	float theta_sq = dot(w, w);
	float theta = sqrt(theta_sq);

	float A, B;

	Matrix3f R; Vector3f T;

	Vector3f crossV = cross(w, t);
	if (theta_sq < 1e-8f)
	{
		A = 1.0f - one_6th * theta_sq; B = 0.5f;
		T.x = t.x + 0.5f * crossV.x; T.y = t.y + 0.5f * crossV.y; T.z = t.z + 0.5f * crossV.z;
	}
	else
	{
		float C;
		if (theta_sq < 1e-6f)
		{
			C = one_6th * (1.0f - one_20th * theta_sq);
			A = 1.0f - theta_sq * C;
			B = 0.5f - 0.25f * one_6th * theta_sq;
		}
		else
		{
			float inv_theta = 1.0f / theta;
			A = sinf(theta) * inv_theta;
			B = (1.0f - cosf(theta)) * (inv_theta * inv_theta);
			C = (1.0f - A) * (inv_theta * inv_theta);
		}

		Vector3f cross2 = cross(w, crossV);

		T.x = t.x + B * crossV.x + C * cross2.x; T.y = t.y + B * crossV.y + C * cross2.y; T.z = t.z + B * crossV.z + C * cross2.z;
	}

	float wx2 = w.x * w.x, wy2 = w.y * w.y, wz2 = w.z * w.z;
	R.m[0 + 3 * 0] = 1.0f - B*(wy2 + wz2);
	R.m[1 + 3 * 1] = 1.0f - B*(wx2 + wz2);
	R.m[2 + 3 * 2] = 1.0f - B*(wx2 + wy2);

	float a, b;
	a = A * w.z, b = B * (w.x * w.y);
	R.m[0 + 3 * 1] = b - a;
	R.m[1 + 3 * 0] = b + a;

	a = A * w.y, b = B * (w.x * w.z);
	R.m[0 + 3 * 2] = b + a;
	R.m[2 + 3 * 0] = b - a;

	a = A * w.x, b = B * (w.y * w.z);
	R.m[1 + 3 * 2] = b - a;
	R.m[2 + 3 * 1] = b + a;

	M.m[0 + 4*0] = R.m[0 + 3*0]; M.m[1 + 4*0] = R.m[1 + 3*0]; M.m[2 + 4*0] = R.m[2 + 3*0];
	M.m[0 + 4*1] = R.m[0 + 3*1]; M.m[1 + 4*1] = R.m[1 + 3*1]; M.m[2 + 4*1] = R.m[2 + 3*1];
	M.m[0 + 4*2] = R.m[0 + 3*2]; M.m[1 + 4*2] = R.m[1 + 3*2]; M.m[2 + 4*2] = R.m[2 + 3*2];

	M.m[0 + 4*3] = T.v[0]; M.m[1 + 4*3] = T.v[1]; M.m[2 + 4*3] = T.v[2];

	M.m[3 + 4*0] = 0.0f; M.m[3 + 4*1] = 0.0f; M.m[3 + 4*2] = 0.0f; M.m[3 + 4*3] = 1.0f;
}

void ITMPose::SetParamsFromModelView()
{
	Vector3f resultRot;
	Matrix3f R = GetR();
	Vector3f T = GetT();

	float cos_angle = (R.m00  + R.m11 + R.m22 - 1.0f) * 0.5f;
	resultRot.x = (R.m[2 + 3 * 1] - R.m[1 + 3 * 2]) * 0.5f;
	resultRot.y = (R.m[0 + 3 * 2] - R.m[2 + 3 * 0]) * 0.5f;
	resultRot.z = (R.m[1 + 3 * 0] - R.m[0 + 3 * 1]) * 0.5f;

	float sin_angle_abs = sqrt(dot(resultRot, resultRot));

	if (cos_angle > M_SQRT1_2)
	{
		if (sin_angle_abs) 
		{
			float p = asinf(sin_angle_abs) / sin_angle_abs;
			resultRot.x *= p; resultRot.y *= p; resultRot.z *= p;
		}
	}
	else
	{
		if (cos_angle > -M_SQRT1_2)
		{
			float p = acosf(cos_angle) / sin_angle_abs;
			resultRot.x *= p; resultRot.y *= p; resultRot.z *= p;
		}
		else
		{
			float angle = (float)M_PI - asinf(sin_angle_abs);
			float d0 = R.m[0 + 3 * 0] - cos_angle;
			float d1 = R.m[1 + 3 * 1] - cos_angle;
			float d2 = R.m[2 + 3 * 2] - cos_angle;

			Vector3f r2;

			if(fabsf(d0) > fabsf(d1) && fabsf(d0) > fabsf(d2))
			{ r2.x = d0; r2.y = (R.m[1 + 3 * 0] + R.m[0 + 3 * 1]) * 0.5f; r2.z = (R.m[0 + 3 * 2] + R.m[2 + 3 * 0]) * 0.5f; } 
			else 
			{
				if(fabsf(d1) > fabsf(d2)) 
				{ r2.x = (R.m[1 + 3 * 0] + R.m[0 + 3 * 1]) * 0.5f; r2.y = d1; r2.z = (R.m[2 + 3 * 1] + R.m[1 + 3 * 2]) * 0.5f; }
				else { r2.x = (R.m[0 + 3 * 2] + R.m[2 + 3 * 0]) * 0.5f; r2.y = (R.m[2 + 3 * 1] + R.m[1 + 3 * 2]) * 0.5f; r2.z = d2; }
			}

			if (dot(r2, resultRot) < 0.0f) { r2.x *= -1.0f; r2.y *= -1.0f; r2.z *= -1.0f; }

			r2 = normalize(r2);

			resultRot.x = angle * r2.x; resultRot.y = angle * r2.y; resultRot.z = angle * r2.z;
		}
	}

	float shtot = 0.5f;
	float theta = sqrt(dot(resultRot, resultRot));

	if (theta > 0.00001f) shtot = sinf(theta * 0.5f) / theta;

	ITMPose halfrotor(0.0f, 0.0f, 0.0f, resultRot.x * -0.5f, resultRot.y * -0.5f, resultRot.z * -0.5f);

	Vector3f rottrans = halfrotor.GetR() * T;

	if (theta > 0.001f)
	{
		float denom = dot(resultRot, resultRot);
		float param = dot(T, resultRot) * (1 - 2 * shtot) / denom;
		
		rottrans.x -= resultRot.x * param; rottrans.y -= resultRot.y * param; rottrans.z -= resultRot.z * param;
	}
	else
	{
		float param = dot(T, resultRot) / 24;
		rottrans.x -= resultRot.x * param; rottrans.y -= resultRot.y * param; rottrans.z -= resultRot.z * param;
	}

	rottrans.x /= 2 * shtot; rottrans.y /= 2 * shtot; rottrans.z /= 2 * shtot;

	this->params.each.rx = resultRot.x; this->params.each.ry = resultRot.y; this->params.each.rz = resultRot.z;
	this->params.each.tx = rottrans.x; this->params.each.ty = rottrans.y; this->params.each.tz = rottrans.z; 
}

ITMPose ITMPose::exp(const Vector6f& tangent)
{
	return ITMPose(tangent);
}

void ITMPose::MultiplyWith(const ITMPose *pose)
{
	M = M * pose->M;
	this->SetParamsFromModelView();
}

Matrix3f ITMPose::GetR(void) const
{
	Matrix3f R;
	R.m[0 + 3*0] = M.m[0 + 4*0]; R.m[1 + 3*0] = M.m[1 + 4*0]; R.m[2 + 3*0] = M.m[2 + 4*0];
	R.m[0 + 3*1] = M.m[0 + 4*1]; R.m[1 + 3*1] = M.m[1 + 4*1]; R.m[2 + 3*1] = M.m[2 + 4*1];
	R.m[0 + 3*2] = M.m[0 + 4*2]; R.m[1 + 3*2] = M.m[1 + 4*2]; R.m[2 + 3*2] = M.m[2 + 4*2];

	return R;
}

Vector3f ITMPose::GetT(void) const
{
	Vector3f T;
	T.v[0] = M.m[0 + 4*3]; T.v[1] = M.m[1 + 4*3]; T.v[2] = M.m[2 + 4*3];

	return T;
}

void ITMPose::GetParams(Vector3f &translation, Vector3f &rotation)
{
	translation.x = this->params.each.tx;
	translation.y = this->params.each.ty;
	translation.z = this->params.each.tz;

	rotation.x = this->params.each.rx;
	rotation.y = this->params.each.ry;
	rotation.z = this->params.each.rz;
}

void ITMPose::SetM(const Matrix4f & src)
{
	M = src;
	SetParamsFromModelView();
}

void ITMPose::SetR(const Matrix3f & R)
{
	M.m[0 + 4*0] = R.m[0 + 3*0]; M.m[1 + 4*0] = R.m[1 + 3*0]; M.m[2 + 4*0] = R.m[2 + 3*0];
	M.m[0 + 4*1] = R.m[0 + 3*1]; M.m[1 + 4*1] = R.m[1 + 3*1]; M.m[2 + 4*1] = R.m[2 + 3*1];
	M.m[0 + 4*2] = R.m[0 + 3*2]; M.m[1 + 4*2] = R.m[1 + 3*2]; M.m[2 + 4*2] = R.m[2 + 3*2];

	SetParamsFromModelView();
}

void ITMPose::SetT(const Vector3f & t)
{
	M.m[0 + 4*3] = t.v[0]; M.m[1 + 4*3] = t.v[1]; M.m[2 + 4*3] = t.v[2];

	SetParamsFromModelView();
}

void ITMPose::SetRT(const Matrix3f & R, const Vector3f & t)
{
	M.m[0 + 4*0] = R.m[0 + 3*0]; M.m[1 + 4*0] = R.m[1 + 3*0]; M.m[2 + 4*0] = R.m[2 + 3*0];
	M.m[0 + 4*1] = R.m[0 + 3*1]; M.m[1 + 4*1] = R.m[1 + 3*1]; M.m[2 + 4*1] = R.m[2 + 3*1];
	M.m[0 + 4*2] = R.m[0 + 3*2]; M.m[1 + 4*2] = R.m[1 + 3*2]; M.m[2 + 4*2] = R.m[2 + 3*2];

	M.m[0 + 4*3] = t.v[0]; M.m[1 + 4*3] = t.v[1]; M.m[2 + 4*3] = t.v[2];

	SetParamsFromModelView();
}

Matrix4f ITMPose::GetInvM(void) const
{
	Matrix4f ret;
	M.inv(ret);
	return ret;
}

void ITMPose::SetInvM(const Matrix4f & invM)
{
	invM.inv(M);
	SetParamsFromModelView();
}

void ITMPose::Coerce(void)
{
	SetParamsFromModelView();
	SetModelViewFromParams();
}

