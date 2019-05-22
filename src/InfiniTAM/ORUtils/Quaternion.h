//
// Created by qkgautier on 8/7/16.
//

#pragma once

#include "Vector.h"
#include "Matrix.h"

namespace ORUtils {

    template<class T>
    static inline Vector4<T> QuaternionFromRotationMatrix( const Matrix3<T>& r)
    {
        Vector4<T> q;

        T trace = r.m00 + r.m11 + r.m22;
        if( trace > 0 )
        {
            T s = 0.5 / sqrt(trace + 1.0);
            q.w = 0.25 / s;
            q.x = ( r.m21 - r.m12 ) * s;
            q.y = ( r.m02 - r.m20 ) * s;
            q.z = ( r.m10 - r.m01 ) * s;
        }
        else
        {
            if ( r.m00 > r.m11 && r.m00 > r.m22 )
            {
                T s = 2.0 * sqrt( 1.0 + r.m00 - r.m11 - r.m22);
                q.w = (r.m21 - r.m12 ) / s;
                q.x = 0.25 * s;
                q.y = (r.m01 + r.m10 ) / s;
                q.z = (r.m02 + r.m20 ) / s;
            }
            else if (r.m11 > r.m22)
            {
                T s = 2.0 * sqrt( 1.0 + r.m11 - r.m00 - r.m22);
                q.w = (r.m02 - r.m20 ) / s;
                q.x = (r.m01 + r.m10 ) / s;
                q.y = 0.25 * s;
                q.z = (r.m12 + r.m21 ) / s;
            }
            else
            {
                T s = 2.0f * sqrt( 1.0 + r.m22 - r.m00 - r.m11 );
                q.w = (r.m10 - r.m01 ) / s;
                q.x = (r.m02 + r.m20 ) / s;
                q.y = (r.m12 + r.m21 ) / s;
                q.z = 0.25 * s;
            }
        }

        return q;
    }

    template<class T>
    static inline Matrix3<T> QuaternionToRotationMatrix( const Vector4<T>& q)
    {
        return Matrix3<T>(1.0-2.0*(q.y*q.y+q.z*q.z), 2.0*(q.x*q.y-q.w*q.z),       2.0*(q.x*q.z+q.w*q.y),
                          2.0*(q.x*q.y+q.w*q.z),     1.0 - 2.0*(q.x*q.x+q.z*q.z), 2.0*(q.y*q.z-q.w*q.x),
                          2.0*(q.x*q.z-q.w*q.y),     2.0*(q.y*q.z+q.w*q.x),       1.0 - 2*(q.x*q.x+q.y*q.y));
    }

    template<class T>
    static T QuaternionLength(const Vector4<T>& q)
    {
        return sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    }

    template<class T>
    static Vector4<T> QuaternionNormalized(const Vector4<T>& q)
    {
        return q / QuaternionLength(q);
    }

    template<class T>
    static Vector4<T> QuaternionLerp(const Vector4<T>& q1, const Vector4<T>& q2, T t)
    {
        Vector4<T> q = QuaternionNormalized(q1*(1.0-t) + q2*t);
        return q;
    }

} // namespace