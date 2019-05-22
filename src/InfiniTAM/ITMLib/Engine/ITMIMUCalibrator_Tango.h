//
// Created by qkgautier on 7/30/16.
//


#pragma once

#include "ITMIMUCalibrator.h"

namespace ITMLib
{
    namespace Objects
    {
        class ITMIMUCalibrator_Tango: public ITMIMUCalibrator
        {
        public:
            ITMIMUCalibrator_Tango(): hasR_(false), hasT_(false), first_(true)
            {
                R_new_.setIdentity();
            }

            virtual void RegisterMeasurement(const Matrix3f & R)
            {
                R_new_ = R;
                hasR_ = true;
                setTransfo();
            }


            virtual Matrix3f GetDifferentialRotationChange()
            {
                return transfo_diff_.GetR();
            }

            virtual void RegisterMeasurement(const Vector3f & t)
            {
                t_new_ = t;
                hasT_ = true;
                setTransfo();
            }

            virtual Vector3f GetDifferentialTranslationChange()
            {
                return transfo_diff_.GetT();
            }

            virtual bool hasTranslation(){ return true; }

        private:
            Matrix3f R_new_; Vector3f t_new_;

            ITMPose transfo_old_, transfo_new_;
            ITMPose transfo_diff_;

            bool hasR_, hasT_;
            bool first_;

        private:
            void setTransfo()
            {
                if(hasR_ && hasT_)
                {
                    transfo_old_ = transfo_new_;
                    transfo_new_.SetRT(R_new_, t_new_);

                    if(first_)
                    {
                        first_ = false;
                        transfo_old_ = transfo_new_;
                    }


                    transfo_diff_.SetM(transfo_old_.GetInvM() * transfo_new_.GetM());

//                    transfo_diff_ = transfo_new_;

                    hasR_ = false;
                    hasT_ = false;
                }
            }

        }; // class ITMIMUCalibrator_Tango

    } // namespace
} // namespace