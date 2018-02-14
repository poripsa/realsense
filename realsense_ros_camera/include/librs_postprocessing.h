// License: Apache 2.0. See LICENSE file in librealsense2 root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include "../include/base_realsense_node.h"
#include "../include/librealsense2/h/rs_option.h"

#ifndef LIBRS_POST_PROCESSING_H
#define LIBRS_POST_PROCESSING_H

// RSPPAPI definition
#ifdef _WIN32
#ifdef RSPPAPI_EXPORTS
#define RSPPAPI __declspec(dllexport)
#else
#define RSPPAPI __declspec(dllimport)
#endif
#else
#define RSPPAPI
#endif


// Filters
#define TEMPORAL_BIT	1
#define SPATIAL_BIT		2


// C APIs
#ifdef __cplusplus
extern "C"
{
#endif

using namespace realsense_ros_camera;

	struct rs2_frame;

	/* Initialization, call once for initialization */
  RSPPAPI void RSPP_Init();

	/* Finalization, call once for clean up */
	RSPPAPI void RSPP_Fini();

	/* Set Parameters, call after initialization and before post processing
	* \param[in] enable The bitwise or to tell which to enable. e.g. "TEMPORAL_BIT | SPATIAL_BIT" means to enable temporal and sptial filters
	* \param[in] downsample_factor Downsample factor as in librs. The down sample scale is "1 << (downsample_factor - 1)"
	* \param[in] spatial_iter Spatial filter iteration number
	* \param[in] spatial_alpha The alpha value in sptial filter
	* \param[in] spatial_delta The delta value in sptial filter
	* \param[out] temporal_table_idx The credible index in temporal filter
	* \param[in] temporal_alpha The alpha value in temporal filter
	* \param[in] temporal_delta The delta value in temporal filter
	*/
	RSPPAPI void SetPostProcessingParams(int enable,
								         int downsample_factor,
								         int spatial_iter, float spatial_alpha, float spatial_delta,
								         int temporal_table_idx, float temporal_alpha, float temporal_delta);

	/* Processing, can be used for each depth frame
	* \param[in] frame The input rs2_frame pointer
	* \return the postprocessed rs2_frame pointer
	*/
	RSPPAPI rs2_frame * PostProcessing(rs2_frame * frame);

#ifdef __cplusplus
}
#endif

#endif
