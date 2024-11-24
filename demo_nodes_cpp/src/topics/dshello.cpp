#include "libdepthsense/ds.hpp"
#include "libdepthsense/dsutils.h"
#include "libdepthsense/dsutils.hpp"
#include <iostream>
#include <sstream>
#include <string>

using std::string;
using std::to_string;

string hello_depthsense() try
{
    // Set minimal log level as DS_LOG_WARN
    // - DS_LOG_ERROR /*!< Critical errors, software module can not recover on its own */
    // - DS_LOG_WARN  /*!< Error conditions from which recovery measures have been taken */
    // - DS_LOG_INFO  /*!< Information messages which describe normal flow of events */
    // - DS_LOG_DEBUG /*!< Extra information which is not necessary for normal use (values, pointers, sizes, etc). */
    ds_set_log_level(DS_LOG_WARN);

    ds::context ctx;
    int w, h;

    // get camera count
    int camera_count = ctx.get_camera_count();

    if (camera_count < 1) {
        throw std::runtime_error("No camera detected. Is it plugged in?");
    }

    std::stringstream ss;
    ss << "camera_count is " << camera_count << "\n";

    // open camera 0
    ds::camera cam = ctx.get_camera(0);

    // print camera name
    ss << "camera name: " << cam.get_name() << "\n";

    // device path [bus:x]-[ports:x.x]-[dev:x]
    ss << "device path: " << cam.get_device_path() << "\n";

    // print serial number
    ss << "SN: " << cam.get_sn() << "\n";

    // enable depth stream: w 640, h 400, 16-bit, 30fps
    cam.enable_stream(DS_STREAM_DEPTH, 640, 400, DS_FORMAT_Z16, 30);

    // start capture with enabled streams
    cam.start_capture();
    
    // set depth quality level as optimized.
    uint8_t depth_level = DS_DEPTH_LEVEL_OPTIMIZED;
    cam.set_option(DS_OPTION_DEPTH_CONTROL_PRESET, &depth_level, sizeof(depth_level));

    // enable auto exposure
    uint8_t ae_enable = 1;
    cam.set_option(DS_OPTION_LR_AUTO_EXPOSURE_ENABLED, &ae_enable, sizeof(ae_enable));

    // enable spot filter
    uint8_t spot_filt_en = 1;
    cam.set_option(DS_OPTION_SPOT_FILT_ENABLED, &spot_filt_en, sizeof(spot_filt_en));

    // set minimum depth distance
    uint32_t min_depth_mm = 260;        // 260   mm
    cam.set_option(DS_OPTION_DEPTH_CLAMP_MIN, &min_depth_mm, sizeof(min_depth_mm));
    
    // set maximum depth distance
    uint32_t max_depth_mm = 10000;      // 10000 mm
    cam.set_option(DS_OPTION_DEPTH_CLAMP_MAX, &max_depth_mm, sizeof(max_depth_mm));

    // get depth image width and height
    w = cam.get_stream_width(DS_STREAM_DEPTH);
    h = cam.get_stream_height(DS_STREAM_DEPTH);
    
    // get intrinsic matrix
    ds::intrinsics intr = cam.get_stream_intrinsics(DS_STREAM_DEPTH);
    // get extrinsic matrix
    ds::extrinsics extr = cam.get_stream_extrinsics(DS_STREAM_INFRARED, DS_STREAM_INFRARED_2);
        
    float hfov, vfov;
    hfov = ds_compute_fov(intr.image_size[0], intr.focal_length[0], intr.principal_point[0]);
    vfov = ds_compute_fov(intr.image_size[1], intr.focal_length[1], intr.principal_point[1]);

    // print out cx, cy, fx, fy and T
    ss << "intr: cx " << intr.principal_point[0] << ", cy " << intr.principal_point[1] << "\n";
    ss << "intr: fx " << intr.focal_length[0] << ", fy " << intr.focal_length[1] << "\n";
    ss << "T is " << extr.translation[0] << "\n";
    ss << "HFOV: " << hfov << ", VFOV: " << vfov << "\n"; 

    ss << "depth stream: w is " << w << ", h is " << h << "\n" ;

    while (1) {
        // wait for all enabled streams
        cam.wait_all_streams();

        // get depth stream data pointer
        ds::Mat depth = cam.get_image_mat(DS_STREAM_DEPTH);
        //
        // TODO: read depth data from depth.data, w(depth.cols), h(depth.rows), 16-bit uint16
        //

        // transfer depth to point cloud
        ds::Mat img_3d;
        // if skip invalid flag is true, skip putting invalid 3d point into img_3d
        bool skip_invalid = true;
        
        // subsample every interval+1 row and every interval+1 column.
        // if interval == 0, no subsample.
        unsigned int interval = 0;
        ds_repoject_image_to_3d(img_3d, depth, intr, skip_invalid, interval);

        //
        // TODO: read data from img_3d.data: 32-bit float x3 (X, Y, Z)
        //  float *point_cloud = (float *)img_3d.data;
        // For each point i:
        //   point_cloud[i*3+0] is X 
        //   point_cloud[i*3+1] is Y 
        //   point_cloud[i*3+2] is Z
        int valid_3d_point_count = img_3d.total() / img_3d.element_size();
        // NOTE: each element of valid_3d_point_count is [X, Y, Z] (Three 32-bit floats)
        ss << "Valid 3D point[X, Y, Z]: count: " << valid_3d_point_count
           << ", frame num: " << img_3d.frame_number()
           << "timestamp: " << img_3d.timestamp_ms() << " ms\n";

        break;
    }

    return ss.str();
}
catch (const std::exception& e)
{
    std::cerr << e.what() << "\\n";
    return string(e.what());
}
