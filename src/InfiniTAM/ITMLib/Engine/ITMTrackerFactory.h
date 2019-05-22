// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <map>
#include <stdexcept>

#include "ITMCompositeTracker.h"
#include "ITMCompositeStepTracker.h"
#include "ITMExternalTracker.h"
#include "ITMIMUTracker.h"
#include "ITMIMUOnlyTracker.h"
#include "ITMLowLevelEngine.h"
#include "ITMTracker.h"

#include "DeviceSpecific/CPU/ITMColorTracker_CPU.h"
#include "DeviceSpecific/CPU/ITMDepthTracker_CPU.h"
#include "DeviceSpecific/CPU/ITMWeightedICPTracker_CPU.h"
#include "DeviceSpecific/CPU/ITMRenTracker_CPU.h"
#include "../Utils/ITMLibSettings.h"

#ifndef COMPILE_WITHOUT_CUDA
#include "DeviceSpecific/CUDA/ITMColorTracker_CUDA.h"
#include "DeviceSpecific/CUDA/ITMDepthTracker_CUDA.h"
#include "DeviceSpecific/CUDA/ITMWeightedICPTracker_CUDA.h"
#include "DeviceSpecific/CUDA/ITMRenTracker_CUDA.h"
#endif

#ifdef COMPILE_WITH_METAL
#include "DeviceSpecific/Metal/ITMDepthTracker_Metal.h"
#endif

#ifdef COMPILE_WITH_OPENCL
#include "DeviceSpecific/OpenCL/ITMDepthTracker_OpenCL.h"
#endif

namespace ITMLib
{
  namespace Engine
  {
    /**
     * \brief An instance of this class can be used to construct trackers.
     */
    template <typename TVoxel, typename TIndex>
    class ITMTrackerFactory
    {
      //#################### TYPEDEFS ####################
    private:
      typedef ITMTracker *(*Maker)(const Vector2i&,const ITMLibSettings*,const ITMLowLevelEngine*,ITMIMUCalibrator*,ITMScene<TVoxel,TIndex>*);

      //#################### PRIVATE VARIABLES ####################
    private:
      /** A map of maker functions for the various different tracker types. */
      std::map<ITMLibSettings::TrackerType,Maker> makers;

      //#################### SINGLETON IMPLEMENTATION ####################
    private:
      /**
       * \brief Constructs a tracker factory.
       */
      ITMTrackerFactory()
      {
        makers.insert(std::make_pair(ITMLibSettings::TRACKER_COLOR, &MakeColourTracker));
        makers.insert(std::make_pair(ITMLibSettings::TRACKER_ICP, &MakeICPTracker));
		makers.insert(std::make_pair(ITMLibSettings::TRACKER_WICP, &MakeWeightedICPTracker));
        makers.insert(std::make_pair(ITMLibSettings::TRACKER_IMU, &MakeIMUTracker));
        makers.insert(std::make_pair(ITMLibSettings::TRACKER_IMU_ONLY, &MakeIMUOnlyTracker));
        makers.insert(std::make_pair(ITMLibSettings::TRACKER_REN, &MakeRenTracker));
        makers.insert(std::make_pair(ITMLibSettings::TRACKER_IMU_INIT, &MakeIMUInitTracker));
        makers.insert(std::make_pair(ITMLibSettings::TRACKER_EXTERNAL_ONLY, &MakeExternalOnlyTracker));
        makers.insert(std::make_pair(ITMLibSettings::TRACKER_EXTERNAL_AND_ICP, &MakeExternalICPTracker));
      }

    public:
      /**
       * \brief Gets the singleton instance for the current set of template parameters.
       */
      static ITMTrackerFactory& Instance()
      {
        static ITMTrackerFactory s_instance;
        return s_instance;
      }

      //#################### PUBLIC MEMBER FUNCTIONS ####################
    public:
      /**
       * \brief Makes a tracker of the type specified in the settings.
       */
      ITMTracker *Make(const Vector2i& trackedImageSize, const ITMLibSettings *settings, const ITMLowLevelEngine *lowLevelEngine,
                       ITMIMUCalibrator *imuCalibrator, ITMScene<TVoxel,TIndex> *scene) const
      {
        typename std::map<ITMLibSettings::TrackerType,Maker>::const_iterator it = makers.find(settings->trackerType);
        if(it == makers.end()) DIEWITHEXCEPTION("Unknown tracker type");

        Maker maker = it->second;
        return (*maker)(trackedImageSize, settings, lowLevelEngine, imuCalibrator, scene);
      }

      //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
    public:
      /**
       * \brief Makes a colour tracker.
       */
      static ITMTracker *MakeColourTracker(const Vector2i& trackedImageSize, const ITMLibSettings *settings, const ITMLowLevelEngine *lowLevelEngine,
                                           ITMIMUCalibrator *imuCalibrator, ITMScene<TVoxel,TIndex> *scene)
      {
        switch(settings->deviceType)
        {
          case ITMLibSettings::DEVICE_CPU:
          case ITMLibSettings::DEVICE_OPENCL:
          {
            return new ITMColorTracker_CPU(trackedImageSize, settings->trackingRegime, settings->noHierarchyLevels, lowLevelEngine);
          }
          case ITMLibSettings::DEVICE_CUDA:
          {
#ifndef COMPILE_WITHOUT_CUDA
            return new ITMColorTracker_CUDA(trackedImageSize, settings->trackingRegime, settings->noHierarchyLevels, lowLevelEngine);
#else
            break;
#endif
          }
          case ITMLibSettings::DEVICE_METAL:
          {
#ifdef COMPILE_WITH_METAL
            return new ITMColorTracker_CPU(trackedImageSize, settings->trackingRegime, settings->noHierarchyLevels, lowLevelEngine);
#else
            break;
#endif
          }
          default: break;
        }

        DIEWITHEXCEPTION("Failed to make colour tracker");
      }

      /**
       * \brief Makes an ICP tracker.
       */
      static ITMTracker *MakeICPTracker(const Vector2i& trackedImageSize, const ITMLibSettings *settings, const ITMLowLevelEngine *lowLevelEngine,
                                        ITMIMUCalibrator *imuCalibrator, ITMScene<TVoxel,TIndex> *scene)
      {
        switch(settings->deviceType)
        {
          case ITMLibSettings::DEVICE_OPENCL:
#ifdef COMPILE_WITH_OPENCL
        	  if(settings->openclAlgo == ITMLibSettings::OPENCL_ALGO_ICP ||
       			   settings->openclAlgo == ITMLibSettings::OPENCL_ALGO_INTEGRATE_RAYCAST_ICP ||
       			   settings->openclAlgo == ITMLibSettings::OPENCL_ALGO_COMBINED_ICP)
        	  {
        		  ITMDepthTracker_OpenCL* tracker = new ITMDepthTracker_OpenCL(
        				  trackedImageSize,
						  settings->trackingRegime,
						  settings->noHierarchyLevels,
						  settings->noICPRunTillLevel,
						  settings->depthTrackerICPThreshold,
						  settings->depthTrackerTerminationThreshold,
						  lowLevelEngine
        		  );
        		  tracker->setNoIterationsPerLevel(settings->noICPIterations);
        		  return tracker;
        	  }
        	  // else fall back to CPU version
#endif
          case ITMLibSettings::DEVICE_CPU:
          {
              ITMDepthTracker_CPU* tracker = new ITMDepthTracker_CPU(
              trackedImageSize,
              settings->trackingRegime,
              settings->noHierarchyLevels,
              settings->noICPRunTillLevel,
              settings->depthTrackerICPThreshold,
              settings->depthTrackerTerminationThreshold,
              lowLevelEngine
              );
              tracker->setNoIterationsPerLevel(settings->noICPIterations);
              return tracker;
          }
          case ITMLibSettings::DEVICE_CUDA:
          {
#ifndef COMPILE_WITHOUT_CUDA
              ITMDepthTracker_CUDA* tracker = new ITMDepthTracker_CUDA(
              trackedImageSize,
              settings->trackingRegime,
              settings->noHierarchyLevels,
              settings->noICPRunTillLevel,
              settings->depthTrackerICPThreshold,
              settings->depthTrackerTerminationThreshold,
              lowLevelEngine
              );
              tracker->setNoIterationsPerLevel(settings->noICPIterations);
              return tracker;
#else
            break;
#endif
          }
          case ITMLibSettings::DEVICE_METAL:
          {
#ifdef COMPILE_WITH_METAL
              ITMDepthTracker_Metal* tracker = new ITMDepthTracker_Metal(
              trackedImageSize,
              settings->trackingRegime,
              settings->noHierarchyLevels,
              settings->noICPRunTillLevel,
              settings->depthTrackerICPThreshold,
              settings->depthTrackerTerminationThreshold,
              lowLevelEngine
              );
              tracker->setNoIterationsPerLevel(settings->noICPIterations);
              return tracker;
#else
            break;
#endif
          }
          default: break;
        }

        DIEWITHEXCEPTION("Failed to make ICP tracker");
      }
	  /**
	  * \brief Makes an WICP tracker.
	  */
	  static ITMTracker *MakeWeightedICPTracker(const Vector2i& trackedImageSize, const ITMLibSettings *settings, const ITMLowLevelEngine *lowLevelEngine,
		  ITMIMUCalibrator *imuCalibrator, ITMScene<TVoxel, TIndex> *scene)
	  {
		  switch (settings->deviceType)
		  {
		  case ITMLibSettings::DEVICE_CPU:
		  case ITMLibSettings::DEVICE_OPENCL:
		  {
			  return new ITMWeightedICPTracker_CPU(
				  trackedImageSize,
				  settings->trackingRegime,
				  settings->noHierarchyLevels,
				  settings->noICPRunTillLevel,
				  settings->depthTrackerICPThreshold,
				  settings->depthTrackerTerminationThreshold,
				  lowLevelEngine
				  );
		  }
		  case ITMLibSettings::DEVICE_CUDA:
		  {
#ifndef COMPILE_WITHOUT_CUDA
			  return new ITMWeightedICPTracker_CUDA(
				  trackedImageSize,
				  settings->trackingRegime,
				  settings->noHierarchyLevels,
				  settings->noICPRunTillLevel,
				  settings->depthTrackerICPThreshold,
				  settings->depthTrackerTerminationThreshold,
				  lowLevelEngine
				  );
#else
			  break;
#endif
		  }
		  case ITMLibSettings::DEVICE_METAL:
		  {
#ifdef COMPILE_WITH_METAL
              ITMDepthTracker_Metal* tracker = new ITMDepthTracker_Metal(
              trackedImageSize,
              settings->trackingRegime,
              settings->noHierarchyLevels,
              settings->noICPRunTillLevel,
              settings->depthTrackerICPThreshold,
              settings->depthTrackerTerminationThreshold,
              lowLevelEngine
              );
              tracker->setNoIterationsPerLevel(settings->noICPIterations);
              return tracker;
#else
			  break;
#endif
		  }
		  default: break;
		  }

		  throw std::runtime_error("Failed to make ICP tracker");
	  }

      /**
       * \brief Makes an IMU tracker.
       */
      static ITMTracker *MakeIMUTracker(const Vector2i& trackedImageSize, const ITMLibSettings *settings, const ITMLowLevelEngine *lowLevelEngine,
                                        ITMIMUCalibrator *imuCalibrator, ITMScene<TVoxel,TIndex> *scene)
      {
        switch(settings->deviceType)
        {
          case ITMLibSettings::DEVICE_OPENCL:
#ifdef COMPILE_WITH_OPENCL
        	  if(settings->openclAlgo == ITMLibSettings::OPENCL_ALGO_ICP ||
       			   settings->openclAlgo == ITMLibSettings::OPENCL_ALGO_INTEGRATE_RAYCAST_ICP ||
       			   settings->openclAlgo == ITMLibSettings::OPENCL_ALGO_COMBINED_ICP)
        	  {
        		  ITMDepthTracker_OpenCL* tracker = new ITMDepthTracker_OpenCL(
        				  trackedImageSize,
						  settings->trackingRegime,
						  settings->noHierarchyLevels,
						  settings->noICPRunTillLevel,
						  settings->depthTrackerICPThreshold,
						  settings->depthTrackerTerminationThreshold,
						  lowLevelEngine
        		  );
        		  tracker->setNoIterationsPerLevel(settings->noICPIterations);
        		  return tracker;
        	  }
        	  // else fall back to CPU version
#endif
          case ITMLibSettings::DEVICE_CPU:
          {
            ITMDepthTracker_CPU* tracker = new ITMDepthTracker_CPU(
                  trackedImageSize,
                  settings->trackingRegime,
                  settings->noHierarchyLevels,
                  settings->noICPRunTillLevel,
                  settings->depthTrackerICPThreshold,
                  settings->depthTrackerTerminationThreshold,
                  lowLevelEngine
            );
            tracker->setNoIterationsPerLevel(settings->noICPIterations);

            ITMCompositeTracker *compositeTracker = new ITMCompositeTracker(2);
            compositeTracker->SetTracker(new ITMIMUTracker(imuCalibrator), 0);
            compositeTracker->SetTracker(tracker, 1);
            return compositeTracker;
          }
          case ITMLibSettings::DEVICE_CUDA:
          {
#ifndef COMPILE_WITHOUT_CUDA
            ITMDepthTracker_CUDA* tracker = new ITMDepthTracker_CUDA(
                  trackedImageSize,
                  settings->trackingRegime,
                  settings->noHierarchyLevels,
                  settings->noICPRunTillLevel,
                  settings->depthTrackerICPThreshold,
                  settings->depthTrackerTerminationThreshold,
                  lowLevelEngine
            );
            tracker->setNoIterationsPerLevel(settings->noICPIterations);

            ITMCompositeTracker *compositeTracker = new ITMCompositeTracker(2);
            compositeTracker->SetTracker(new ITMIMUTracker(imuCalibrator), 0);
            compositeTracker->SetTracker(tracker, 1);
            return compositeTracker;
#else
            break;
#endif
          }
          case ITMLibSettings::DEVICE_METAL:
          {
#ifdef COMPILE_WITH_METAL
            ITMDepthTracker_Metal* tracker = new ITMDepthTracker_Metal(
            trackedImageSize,
            settings->trackingRegime,
            settings->noHierarchyLevels,
            settings->noICPRunTillLevel,
            settings->depthTrackerICPThreshold,
            settings->depthTrackerTerminationThreshold,
            lowLevelEngine
            );
            tracker->setNoIterationsPerLevel(settings->noICPIterations);

            ITMCompositeTracker *compositeTracker = new ITMCompositeTracker(2);
            compositeTracker->SetTracker(new ITMIMUTracker(imuCalibrator), 0);
            compositeTracker->SetTracker(tracker, 1);
            return compositeTracker;
#else
            break;
#endif
          }
          default: break;
        }

        DIEWITHEXCEPTION("Failed to make IMU tracker");
      }


      /**
       * \brief Makes an IMU only tracker.
       */
      static ITMTracker *MakeIMUOnlyTracker(const Vector2i& trackedImageSize, const ITMLibSettings *settings, const ITMLowLevelEngine *lowLevelEngine,
                                            ITMIMUCalibrator *imuCalibrator, ITMScene<TVoxel,TIndex> *scene)
      {
        return new ITMIMUOnlyTracker(imuCalibrator);
        DIEWITHEXCEPTION("Failed to make IMU tracker");
      }



        /**
         * \brief Makes an IMU only tracker.
         */
        static ITMTracker *MakeExternalOnlyTracker(const Vector2i& trackedImageSize, const ITMLibSettings *settings, const ITMLowLevelEngine *lowLevelEngine,
                                                   ITMIMUCalibrator *imuCalibrator, ITMScene<TVoxel,TIndex> *scene)
        {
            return new ITMExternalTracker();
            DIEWITHEXCEPTION("Failed to make External tracker");
        }


        /**
       * \brief Makes a Ren tracker.
       */
      static ITMTracker *MakeRenTracker(const Vector2i& trackedImageSize, const ITMLibSettings *settings, const ITMLowLevelEngine *lowLevelEngine,
                                        ITMIMUCalibrator *imuCalibrator, ITMScene<TVoxel,TIndex> *scene)
      {
        switch(settings->deviceType)
        {
          case ITMLibSettings::DEVICE_CPU:
          case ITMLibSettings::DEVICE_OPENCL:
          {
            return new ITMRenTracker_CPU<TVoxel, TIndex>(
              trackedImageSize,
              settings->trackingRegime,
              2,
              lowLevelEngine, scene
            );
          }
          case ITMLibSettings::DEVICE_CUDA:
          {
#ifndef COMPILE_WITHOUT_CUDA
            return new ITMRenTracker_CUDA<TVoxel, TIndex>(
              trackedImageSize,
              settings->trackingRegime,
              2,
              lowLevelEngine,
              scene
            );
#else
            break;
#endif
          }
          case ITMLibSettings::DEVICE_METAL:
          {
#ifdef COMPILE_WITH_METAL
            return new ITMRenTracker_CPU<TVoxel, TIndex>(
              trackedImageSize,
              settings->trackingRegime,
              2,
              lowLevelEngine, scene
            );
#else
            break;
#endif
          }
          default: break;
        }

        DIEWITHEXCEPTION("Failed to make Ren tracker");
      }


        /**
         * \brief Makes a tracker with an initialization step and an ICP step.
         */
        static ITMTracker *MakeCompositeStepICPTracker(const Vector2i& trackedImageSize, const ITMLibSettings *settings, const ITMLowLevelEngine *lowLevelEngine,
                                                       ITMIMUCalibrator *imuCalibrator, ITMScene<TVoxel,TIndex> *scene, bool imuinit = true)
        {
            switch(settings->deviceType)
            {
               case ITMLibSettings::DEVICE_OPENCL:
#ifdef COMPILE_WITH_OPENCL
            	   if(settings->openclAlgo == ITMLibSettings::OPENCL_ALGO_ICP ||
            			   settings->openclAlgo == ITMLibSettings::OPENCL_ALGO_INTEGRATE_RAYCAST_ICP ||
            			   settings->openclAlgo == ITMLibSettings::OPENCL_ALGO_COMBINED_ICP)
            	   {
            		   ITMDepthTracker_OpenCL* tracker = new ITMDepthTracker_OpenCL(
            				   trackedImageSize,
							   settings->trackingRegime,
							   settings->noHierarchyLevels,
							   settings->noICPRunTillLevel,
							   settings->depthTrackerICPThreshold,
							   settings->depthTrackerTerminationThreshold,
							   lowLevelEngine
            		   );
            		   tracker->setNoIterationsPerLevel(settings->noICPIterations);
            		   return tracker;
            	   }
            	   // else fall back to CPU version
#endif
                case ITMLibSettings::DEVICE_CPU:
                {
                    ITMDepthTracker_CPU* tracker = new ITMDepthTracker_CPU(
                            trackedImageSize,
                            settings->trackingRegime,
                            settings->noHierarchyLevels,
                            settings->noICPRunTillLevel,
                            settings->depthTrackerICPThreshold,
                            settings->depthTrackerTerminationThreshold,
                            lowLevelEngine
                    );
                    tracker->setNoIterationsPerLevel(settings->noICPIterations);

                    ITMCompositeStepTracker *compositeTracker = new ITMCompositeStepTracker(2);
                    if(imuinit){ compositeTracker->SetTracker(new ITMIMUOnlyTracker(imuCalibrator), 0); }
                    else{ compositeTracker->SetTracker(new ITMExternalTracker(), 0);}
                    compositeTracker->SetTracker(tracker, 1);
                    compositeTracker->SetCurrentStep(1);
                    return compositeTracker;
                }
                case ITMLibSettings::DEVICE_CUDA:
                {
#ifndef COMPILE_WITHOUT_CUDA
                    ITMDepthTracker_CUDA* tracker = new ITMDepthTracker_CUDA(
                            trackedImageSize,
                            settings->trackingRegime,
                            settings->noHierarchyLevels,
                            settings->noICPRunTillLevel,
                            settings->depthTrackerICPThreshold,
                            settings->depthTrackerTerminationThreshold,
                            lowLevelEngine
                    );
                    tracker->setNoIterationsPerLevel(settings->noICPIterations);

                    ITMCompositeStepTracker *compositeTracker = new ITMCompositeStepTracker(2);
                    if(imuinit){ compositeTracker->SetTracker(new ITMIMUOnlyTracker(imuCalibrator), 0); }
                    else{ compositeTracker->SetTracker(new ITMExternalTracker(), 0);}
                    compositeTracker->SetTracker(tracker, 1);
                    compositeTracker->SetCurrentStep(1);
                    return compositeTracker;
#else
                    break;
#endif
                }
                case ITMLibSettings::DEVICE_METAL:
                {
#ifdef COMPILE_WITH_METAL
                    ITMDepthTracker_Metal* tracker = new ITMDepthTracker_Metal(
                    trackedImageSize,
                    settings->trackingRegime,
                    settings->noHierarchyLevels,
                    settings->noICPRunTillLevel,
                    settings->depthTrackerICPThreshold,
                    settings->depthTrackerTerminationThreshold,
                    lowLevelEngine
                    );
                    tracker->setNoIterationsPerLevel(settings->noICPIterations);

                    ITMCompositeStepTracker *compositeTracker = new ITMCompositeStepTracker(2);
                    if(imuinit){ compositeTracker->SetTracker(new ITMIMUOnlyTracker(imuCalibrator), 0); }
                    else{ compositeTracker->SetTracker(new ITMExternalTracker(), 0);}
                    compositeTracker->SetTracker(tracker, 1);
                    compositeTracker->SetCurrentStep(1);
                    return compositeTracker;
#else
                    break;
#endif
                }
                default: break;
            }

            DIEWITHEXCEPTION("Failed to make IMU tracker");
        }


        /**
         * \brief Makes an IMU/ICP tracker.
         */
        static ITMTracker *MakeIMUInitTracker(const Vector2i& trackedImageSize, const ITMLibSettings *settings, const ITMLowLevelEngine *lowLevelEngine,
                                              ITMIMUCalibrator *imuCalibrator, ITMScene<TVoxel,TIndex> *scene)
        {
            return MakeCompositeStepICPTracker(trackedImageSize, settings, lowLevelEngine, imuCalibrator, scene, true);
        }


        /**
         * \brief Makes an External/ICP tracker.
         */
        static ITMTracker *MakeExternalICPTracker(const Vector2i& trackedImageSize, const ITMLibSettings *settings, const ITMLowLevelEngine *lowLevelEngine,
                                                  ITMIMUCalibrator *imuCalibrator, ITMScene<TVoxel,TIndex> *scene)
        {
            return MakeCompositeStepICPTracker(trackedImageSize, settings, lowLevelEngine, imuCalibrator, scene, false);
        }


    }; // class ITMTrackerFactory
  } // namespace
} // namespace


