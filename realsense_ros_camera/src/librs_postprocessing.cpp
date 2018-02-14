#include "librs_postprocessing.h"
#include <librealsense2/rs.h>

#include <iostream>
#define check_error(e) if (e) std::cout << rs2_get_error_message(e) << std::endl;


// Globals
static int g_enable = true;
static int g_downSampleScale = 1;

static rs2_processing_block * g_decimation_filter_block = nullptr;
static rs2_frame_queue * g_decimation_filter_queue = nullptr;

static rs2_processing_block * g_disparity_transforg_1_block = nullptr;
static rs2_frame_queue * g_disparity_transforg_1_queue = nullptr;

static rs2_processing_block * g_spatial_filter_block = nullptr;
static rs2_frame_queue * g_spatial_filter_queue = nullptr;

static rs2_processing_block * g_temporal_filter_block = nullptr;
static rs2_frame_queue * g_temporal_filter_queue = nullptr;

static rs2_processing_block * g_disparity_transforg_2_block = nullptr;
static rs2_frame_queue * g_disparity_transforg_2_queue = nullptr;

static rs2_error * e = nullptr;


// Initialization, call once for initialization
void RSPP_Init()
{
  // downsample
  g_decimation_filter_queue = rs2_create_frame_queue(1, &e);
  check_error(e);

  g_decimation_filter_block = rs2_create_decimation_filter_block(&e);
  check_error(e);

  rs2_start_processing_queue(g_decimation_filter_block, g_decimation_filter_queue, &e);
  check_error(e);

  // depth to disparity
  g_disparity_transforg_1_queue = rs2_create_frame_queue(1, &e);
  check_error(e);

  g_disparity_transforg_1_block = rs2_create_disparity_transform_block(true, &e);
  check_error(e);

  rs2_start_processing_queue(g_disparity_transforg_1_block, g_disparity_transforg_1_queue, &e);
  check_error(e);

  // spatial
  g_spatial_filter_queue = rs2_create_frame_queue(1, &e);
  check_error(e);

  g_spatial_filter_block = rs2_create_spatial_filter_block(&e);
  check_error(e);

  rs2_start_processing_queue(g_spatial_filter_block, g_spatial_filter_queue, &e);
  check_error(e);

  // temporal
  g_temporal_filter_queue = rs2_create_frame_queue(1, &e);
  check_error(e);

  g_temporal_filter_block = rs2_create_temporal_filter_block(&e);
  check_error(e);

  rs2_start_processing_queue(g_temporal_filter_block, g_temporal_filter_queue, &e);
  check_error(e);

  // diparity to depth
  g_disparity_transforg_2_queue = rs2_create_frame_queue(1, &e);
  check_error(e);

  g_disparity_transforg_2_block = rs2_create_disparity_transform_block(false, &e);
  check_error(e);

  rs2_start_processing_queue(g_disparity_transforg_2_block, g_disparity_transforg_2_queue, &e);
  check_error(e);
}


// Finalization, call once for clena up
void RSPP_Fini()
{
  // diparity to depth
  rs2_delete_processing_block(g_disparity_transforg_2_block);
  g_disparity_transforg_2_block = nullptr;

  rs2_delete_frame_queue(g_disparity_transforg_2_queue);
  g_disparity_transforg_2_queue = nullptr;

  // temporal
  rs2_delete_processing_block(g_temporal_filter_block);
  g_temporal_filter_block = nullptr;

  rs2_delete_frame_queue(g_temporal_filter_queue);
  g_temporal_filter_queue = nullptr;

  // spatial
  rs2_delete_processing_block(g_spatial_filter_block);
  g_spatial_filter_block = nullptr;

  rs2_delete_frame_queue(g_spatial_filter_queue);
  g_spatial_filter_queue = nullptr;

  // depth to disparity
  rs2_delete_processing_block(g_disparity_transforg_1_block);
  g_disparity_transforg_1_block = nullptr;

  rs2_delete_frame_queue(g_disparity_transforg_1_queue);
  g_disparity_transforg_1_queue = nullptr;

  // downsample
  rs2_delete_processing_block(g_decimation_filter_block);
  g_decimation_filter_block = nullptr;

  rs2_delete_frame_queue(g_decimation_filter_queue);
  g_decimation_filter_block = nullptr;
}


// Set Parameters
void SetPostProcessingParams(int enable,
               int downsample_factor,
               int spatial_iter, float spatial_alpha, float spatial_delta,
               int temporal_table_idx, float temporal_alpha, float temporal_delta)
{
  // enable
  g_enable = enable;

  // downsample
  g_downSampleScale = 1 << (downsample_factor - 1);
  rs2_set_option((rs2_options *)g_decimation_filter_block, RS2_OPTION_FILTER_MAGNITUDE, (float)downsample_factor, &e);
  check_error(e);

  // spatial
  rs2_set_option((rs2_options *)g_spatial_filter_block, RS2_OPTION_FILTER_MAGNITUDE, (float)spatial_iter, &e);
  check_error(e);

  rs2_set_option((rs2_options *)g_spatial_filter_block, RS2_OPTION_FILTER_SMOOTH_ALPHA, spatial_alpha, &e);
  check_error(e);

  rs2_set_option((rs2_options *)g_spatial_filter_block, RS2_OPTION_FILTER_SMOOTH_DELTA, spatial_delta, &e);
  check_error(e);

  // temporal
  rs2_set_option((rs2_options *)g_temporal_filter_block, RS2_OPTION_HOLES_FILL, (float)temporal_table_idx, &e);
  check_error(e);

  rs2_set_option((rs2_options *)g_temporal_filter_block, RS2_OPTION_FILTER_SMOOTH_ALPHA, temporal_alpha, &e);
  check_error(e);

  rs2_set_option((rs2_options *)g_temporal_filter_block, RS2_OPTION_FILTER_SMOOTH_DELTA, temporal_delta, &e);
  check_error(e);
}


// Process, used in PostProcessing function
rs2_frame * Process(rs2_processing_block * block, rs2_frame_queue * queue, rs2_frame * frame)
{
  rs2_process_frame(block, frame, &e);
  check_error(e);

  rs2_frame * f = rs2_wait_for_frame(queue, 100, &e);
  check_error(e);

  return f;
}


// PostProcessing, can be used for each depth frame after initialization
rs2_frame * PostProcessing(rs2_frame * frame)
{
  // downsample
  if (g_downSampleScale > 1)
    frame = Process(g_decimation_filter_block, g_decimation_filter_queue, frame);

  // depth to disparity
  if (g_enable & (TEMPORAL_BIT | SPATIAL_BIT))
    frame = Process(g_disparity_transforg_1_block, g_disparity_transforg_1_queue, frame);

  // spatial
  if (g_enable & SPATIAL_BIT)
    frame = Process(g_spatial_filter_block, g_spatial_filter_queue, frame);

  // temporal
  if (g_enable & TEMPORAL_BIT)
    frame = Process(g_temporal_filter_block, g_temporal_filter_queue, frame);

  // disparity to depth
  if (g_enable & (TEMPORAL_BIT | SPATIAL_BIT))
    frame = Process(g_disparity_transforg_2_block, g_disparity_transforg_2_queue, frame);

  return frame;
}

