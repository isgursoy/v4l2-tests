#ifndef ABSTRACT_CAPTURE_BACKEND_HPP
#define ABSTRACT_CAPTURE_BACKEND_HPP

#include <vector>
#include <span>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <array>
#include <cstddef>

namespace Cartrack
{

using Data_Type = std::byte;
static const std::size_t Alignment_Size=128;

#ifdef BOOST_ALIGN_AVAILABLE
using Aligned_Allocator = boost::alignment::aligned_allocator<Data_Type, Alignment_Size>;
using Aligned_Buffer		= std::vector<Data_Type, Aligned_Allocator>;
#else
using Aligned_Buffer = std::vector<Data_Type>;
#endif
using Multiplanar_Buffer = std::vector<Aligned_Buffer>;
#ifdef OCL_AVAILABLE
using Multiplanar_CL_Buffer = std::vector<cl::Buffer>;
#endif
using Multiplanar_Buffer_View = std::vector<std::span<Data_Type>>;

enum class Pixel_Format : uint {
		Invalid = 0,
		YUYV422,
		NV12,
		NV12sp,
		YUV422P,
		MJPEG,
		BGR24,
		RGB24
};
using Camera_ID = short;
static inline long long _timeout_in_milli = 200;

static const inline std::unordered_map<Pixel_Format, std::array<char, 4>>
		pixel_formats_fourcc{
												 {Pixel_Format::YUYV422, {'Y', 'U', 'Y', 'V'}},
												 {Pixel_Format::YUV422P, {'4', '2', '2', 'P'}},
												 {Pixel_Format::NV12, {'N', 'V', '1', '2'}},
												 {Pixel_Format::NV12sp, {'N', 'M', '1', '2'}},
												 {Pixel_Format::MJPEG, {'M', 'J', 'P', 'G'}},
												 {Pixel_Format::BGR24, {'B', 'G', 'R', '3'}},
												 {Pixel_Format::RGB24, {'R', 'G', 'B', '3'}},
												 };


struct Stream_Configuration
{
	public:
		/**
				 * @brief OpenCV backends always do at least 1 mmap copy to isolate internal
				 * buffer.For nonrgb frames, this copy is also converting multiplanar
				 * buffer to single plane buffer. OpenCV will always return deinterleaved
				 *  single plane frame.
				 */
		enum class Capture_Backends {
				isgursoy_V4L2 = 0 // Fastest, zero copy and minimum overhead
#ifdef OCV_AVAILABLE
				,
				OpenCV_V4L2 // CAP_V4L2, next fastest, 1 extra memcpy
				,
				OpenCV_GSTREAMER // CAP_GSTREAMER, totally depends your pipeline,
				// roughly same efficieny with OpenCV_FFmpeg in best case
				,
				OpenCV_FFmpeg // CAP_FFMPEG, slower than OpenCV_V4L2
#endif
		};

	public:
		Capture_Backends backend = Capture_Backends::isgursoy_V4L2;
		enum class Buffering { Invalid = 0, Internal, USERPTR };
		/**
				 * @brief width and height of the desired resolution.
				 *
				 * MANDATORY.
				 */
		uint32_t width = 0, height = 0;
		/**
				 * @brief fps is the desired frames per second. The actual fps will be
				 * determined by the driver. So you need to check the actual fps after
				 * opening the device. Use get_fps getter. If you set fps too high, the
				 * driver will use max it can provide for given pixel format and resolution.
				 *
				 * MANDATORY.
				 */
		uint32_t fps = 0;
		/**
				 * @brief px_format is the desired pixel format.
				 * Consult your v4l2-ctl info.
				 *
				 * MANDATORY.
				 */
		Pixel_Format pixel_format = Pixel_Format::NV12;

		/**
				* @brief device_index should be unique. V4L2 does not permit opening the
				* same device twice.
				*
				* MANDATORY.
				*/
		Camera_ID device_index = -1;

		/**
				 * @brief num_max_internal_buffers num_max_buffers is the maximum number
				 * of buffers that will be allocated. The number of buffers that will be
				 * allocated is determined by the driver and/or OpenCV backend.
				 *
				 * Minimum value is 1 for v4l2 and 0 for OpenCV.
				 * Maximum value is 32 for v4l2 and 10 for OpenCV.
				 */
		unsigned short num_buffers = 1;

		Buffering buffering = Buffering::Internal;

		struct V4L2
		{
			public:
				/**
						 * @brief There will be almost always multiple amount of buffers in use for
						 * internal buffering.
						 * V4L2 will use all buffers that it wants. At the time of get_frame call
						 * either you will get the oldest buffer as usual behaviour or you will
						 * get the newest one by dropping all the previous ones to avoid latency.
						 */
				enum class Internal_Buffering_Strategy { Oldest = 0, Only_Newest };

			public:
				/**
						 * @brief crop_rect is the desired crop rectangle. maps to v4l2_rect.
						 * Follow terminal info during device initialization regarding this capability.
						 */
				std::array<unsigned int, 4> crop_rect = {0, 0, 0, 0};

				Internal_Buffering_Strategy buffer_usage_policy =
						Internal_Buffering_Strategy::Oldest;
				/**
						 * @brief Your desired px_format decides that. See px format documentation.
						 * Consult your driver.
						 */
				bool contiguous = true;

		} v4l2;

#ifdef OCV_VIDEOIO_AVAILABLE
		/**
				 * @brief OpenCV FFmpeg backend can be configured using the environment
				 * variable OPENCV_FFMPEG_CAPTURE_OPTIONS.
				 *
				 * Examples:
				 * OPENCV_FFMPEG_CAPTURE_OPTIONS="hw_decoders_any;vaapi,vdpau"
				 * OPENCV_FFMPEG_CAPTURE_OPTIONS="hw_decoders_any;cuda|vsync;0"
				 * OPENCV_FFMPEG_CAPTURE_OPTIONS="option;value|option;value"
				 */
		struct OpenCV
		{
			public:
				/**
						 * @brief gst_pipeline should be a valid GStreamer pipeline and should
						 * contain the device_index. System will not use device_index of outer
						 * scope.
						 *
						 * Almost none of the setters would work for network stream.
						 *
						 * Example:
						 * gst_pipeline="v4l2src device=/dev/video0 ! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! videoconvert ! appsink"
						 */
				std::string gst_pipeline;

				/**
						 * @brief ffmpeg_file_or_network_stream_url will be used instead
						 * of device_index if not empty.
						 *
						 * Almost none of the setters would work for network stream.
						 *
						 * You can set the number string X of /dev/videoX or any other valid v4l2 device as well.
						 */
				std::string ffmpeg_file_or_network_stream_url;
		} opencv;
#endif

#ifdef OCL_AVAILABLE
		OpenCL_Configuration opencl;
#endif
};

class Capture_Backend
{
	public:
		virtual ~Capture_Backend() = default;

	public:
		virtual Multiplanar_Buffer_View get_frame_data() = 0;

#ifdef OCL_AVAILABLE
		virtual typename Frame_Impl::Multiplanar_CL_Buffer share_frame_data(bool rw = false) = 0;
		[[nodiscard]] virtual std::vector<std::vector<size_t>> put_frame_data(
				std::vector<typename Frame_Impl::Multiplanar_CL_Buffer>& userspace) = 0;
#endif
		[[nodiscard]] virtual std::vector<std::vector<size_t>> put_frame_data(
				std::vector<Multiplanar_Buffer_View>& userspace) = 0;

	public:
		[[nodiscard]] virtual const Stream_Configuration& configuration() const
		{
				return _configuration_;
		}
		[[nodiscard]] virtual Pixel_Format get_pixel_format() const = 0;

		virtual bool set_zoom(int val) = 0;

		[[nodiscard]] virtual int get_zoom() const = 0;

		virtual bool set_focus(int val) = 0;

		[[nodiscard]] virtual int get_focus() const = 0;

		virtual bool set_sharpness(int val) = 0;

		[[nodiscard]] virtual int get_sharpness() const = 0;

		virtual bool set_auto_focus(bool enable) = 0;

		[[nodiscard]] virtual bool get_auto_focus() const = 0;

		virtual bool set_brightness(int val) = 0;

		[[nodiscard]] virtual int get_brightness() const = 0;

		virtual bool set_contrast(int val) = 0;

		[[nodiscard]] virtual int get_contrast() const = 0;

		virtual bool set_saturation(int val) = 0;

		[[nodiscard]] virtual int get_saturation() const = 0;

		virtual bool set_hue(int val) = 0;

		[[nodiscard]] virtual int get_hue() const = 0;

		virtual bool set_gain(int val) = 0;

		[[nodiscard]] virtual int get_gain() const = 0;

		virtual bool set_exposure(int val) = 0;

		[[nodiscard]] virtual int get_exposure() const = 0;

		virtual bool set_white_balance_temperature(int val) = 0;

		[[nodiscard]] virtual int get_white_balance_temperature() const = 0;

		[[nodiscard]] virtual bool get_auto_white_balance_val() const = 0;

		virtual bool set_auto_white_balance(bool enable) = 0;

		virtual bool set_auto_exposure_mode(int type) = 0;

		[[nodiscard]] virtual int get_auto_exposure_current_value() const = 0;

		virtual bool enable_auto_exposure_auto_priority_mode(bool on) = 0;

		[[nodiscard]] virtual bool is_auto_exposure_auto_priority_enabled() const = 0;

		virtual bool set_manual_exposure_value(int val) = 0;

		[[nodiscard]] virtual int get_manual_exposure_value() const = 0;

		[[nodiscard]] virtual double set_fps(double new_fps) = 0;

		[[nodiscard]] virtual double get_fps() const = 0;

		[[nodiscard]] virtual uintmax_t get_frame_order() const
		{
				return _frame_order_;
		}

		[[nodiscard]] virtual unsigned int get_width() const = 0;

		[[nodiscard]] virtual unsigned int get_height() const = 0;

		[[nodiscard]] virtual constexpr size_t num_planes() const = 0;

	protected:
		Stream_Configuration _configuration_;
		uintmax_t _frame_order_ = 0;
};

} // namespace Cartrack

#endif
