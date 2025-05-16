#include "isgursoy_V4L2.hpp"
#include <chrono>
#include <filesystem>
#include <algorithm>
#ifdef LIBPNG_AVAILABLE
#		include <png.h>
#endif

constexpr ushort
get_num_planes(
		Cartrack::Pixel_Format px_format)
{
		switch(px_format)
		{
				case Cartrack::Pixel_Format::NV12sp:
						return 2;
				case Cartrack::Pixel_Format::YUV422P:
						return 3;
				case Cartrack::Pixel_Format::NV12:
				case Cartrack::Pixel_Format::YUYV422:
				case Cartrack::Pixel_Format::MJPEG:
				case Cartrack::Pixel_Format::BGR24:
				case Cartrack::Pixel_Format::RGB24:
						return 1;
				case Cartrack::Pixel_Format::Invalid:
				default:
						return 0;
		}
		return 0;
}

std::vector<size_t>
get_plane_dimensions(
		Cartrack::Pixel_Format px_format,
		size_t width,
		size_t height)
{
		std::vector<size_t> sizes;
		sizes.resize(get_num_planes(px_format));
		switch(px_format)
		{
				case Cartrack::Pixel_Format::MJPEG:
				case Cartrack::Pixel_Format::YUYV422:
						sizes[0] = width * height * 2;
						break;
				case Cartrack::Pixel_Format::YUV422P:
						sizes[0] = width * height;
						sizes[1] = width * height / 2;
						sizes[2] = width * height / 2;
						break;
				case Cartrack::Pixel_Format::NV12:
						sizes[0] = width * height * 1.5;
						break;
				case Cartrack::Pixel_Format::NV12sp:
						sizes[0] = width * height;
						sizes[1] = width * height / 2;
						break;
				case Cartrack::Pixel_Format::BGR24:
				case Cartrack::Pixel_Format::RGB24:
						sizes[0] = width * height * 3;
						break;
				case Cartrack::Pixel_Format::Invalid:
				default:
						break;
		}
		return sizes;
}

using Frame_Plane = std::vector<Cartrack::Data_Type>;
using Frame_Impl	= std::vector<Frame_Plane>;
using uchar				= unsigned char;

void
write_frame_to_disk(
		Cartrack::Multiplanar_Buffer_View& frame,
		int w,
		int h,
		int order,
		bool mmap)
{
		auto convert_nv12_to_bgr = [](const Cartrack::Multiplanar_Buffer_View& data,
																	Frame_Impl& bgr_buffer,
																	size_t width,
																	size_t height,
																	bool rgb)
		{
				const uchar* y_plane = (uchar*) data[0].data();
				const uchar* uv_plane =
						data.size() > 1 ? (uchar*) data[1].data() : (uchar*) y_plane + width * height;
				uchar* bgr = (uchar*) (bgr_buffer[0].data());

				const unsigned int half_width = width / 2;
				for(unsigned short y = 0; y < height; ++y)
				{
						const unsigned int y_stride	 = y * width;
						const unsigned int uv_stride = y / 2 * half_width;
						unsigned int uv_index				 = uv_stride;
						for(unsigned short x = 0; x < width; ++x)
						{
								const unsigned int y_index = 3 * (y_stride + x);
								if(x % 2 == 0)
								{
										uv_index = (uv_stride + x / 2) * 2;
								}

								const short Y = y_plane[y_index / 3];
								const short U = uv_plane[uv_index] - 128;
								const short V = uv_plane[uv_index + 1] - 128;

								bgr[y_index + 1] = std::clamp(
										static_cast<int>(1.164 * (Y - 16) - 0.813 * V - 0.391 * U), 0, 255);
								if(rgb)
								{
										bgr[y_index + 2] =
												std::clamp(static_cast<int>(1.164 * (Y - 16) + 2.018 * U), 0, 255);

										bgr[y_index] =
												std::clamp(static_cast<int>(1.164 * (Y - 16) + 1.596 * V), 0, 255);
								}
								else
								{
										bgr[y_index] =
												std::clamp(static_cast<int>(1.164 * (Y - 16) + 2.018 * U), 0, 255);

										bgr[y_index + 2] =
												std::clamp(static_cast<int>(1.164 * (Y - 16) + 1.596 * V), 0, 255);
								}
						}
				}

				return;
		};


#ifdef LIBPNG_AVAILABLE
		auto save_rgb_png = [](const std::string& filename,
													 int width,
													 int height,
													 std::span<Cartrack::Data_Type> rgb_data) -> int
		{
				FILE* fp = fopen(filename.c_str(), "wb");
				if(!fp)
				{
						std::cerr << "Error opening file for writing: " << filename << std::endl;
						return false;
				}

				png_structp png_ptr =
						png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
				if(!png_ptr)
				{
						fclose(fp);
						std::cerr << "Error creating PNG write structure" << std::endl;
						return

								false;
				}

				png_infop info_ptr = png_create_info_struct(png_ptr);
				if(!info_ptr)
				{
						png_destroy_write_struct(&png_ptr, nullptr);
						fclose(fp);
						std::cerr << "Error creating PNG info structure" << std::endl;
						return false;
				}

				if(setjmp(png_jmpbuf(png_ptr)))
				{
						png_destroy_write_struct(&png_ptr, &info_ptr);
						fclose(fp);
						std::cerr << "Error writing PNG file" << std::endl;
						return false;
				}

				png_init_io(png_ptr, fp);

				png_set_IHDR(png_ptr,
										 info_ptr,
										 width,
										 height,
										 8,
										 PNG_COLOR_TYPE_RGB,
										 PNG_INTERLACE_NONE,
										 PNG_COMPRESSION_TYPE_BASE,
										 PNG_FILTER_TYPE_BASE);

				std::vector<png_bytep> row_pointers(height);
				for(png_uint_32 y = 0; y < height; ++y)
				{
						row_pointers[y] =
								const_cast<png_bytep>((unsigned char*) rgb_data.data() + y * width * 3);
				}

				png_write_info(png_ptr, info_ptr);
				png_write_image(png_ptr, row_pointers.data());
				png_write_end(png_ptr, nullptr);

				png_destroy_write_struct(&png_ptr, &info_ptr);
				fclose(fp);

				return true;
		};
#endif

		Frame_Impl rgb_buffer;
		rgb_buffer.resize(1);
		rgb_buffer[0].resize(w * h * 3);

		convert_nv12_to_bgr(frame, rgb_buffer, w, h, true);

		std::string filename = "cartrack_" + std::string(mmap ? "mmap" : "userptr") + "test_frame_"
													 + std::to_string(order) + ".png";
		save_rgb_png(filename, w, h, rgb_buffer[0]);
}

static void
userptr_capture(
		std::shared_ptr<Cartrack::V4L2_Backend> backend,
		uint num_frames = 1000)
{
		if(backend->configuration().buffering
			 == Cartrack::Stream_Configuration::Buffering::Internal)
		{
				std::cerr
						<< "You are requesting USERPTR frame from MMAP configured camera. This " "will " "cause " "copying data from internal buffer to user space."
						<< std::endl;
		}

		auto px_format	 = backend->get_pixel_format();
		auto num_planes	 = get_num_planes(px_format);
		const int width	 = backend->get_width();
		const int height = backend->get_height();
		auto plane_dims	 = get_plane_dimensions(px_format, width, height);

		auto make_empty_frame = [backend, &px_format, &num_planes, &plane_dims]()
		{
				std::vector<Frame_Plane> allocated_cpu_data;
				allocated_cpu_data.resize(get_num_planes(px_format));
				for(auto plane_index = 0; plane_index < num_planes; ++plane_index)
				{
						allocated_cpu_data[plane_index].resize(plane_dims.at(plane_index));
				}
				return allocated_cpu_data;
		};

		const auto num_buffers = backend->configuration().num_buffers;
		std::vector<Frame_Impl> userspace_frames;
		std::vector<Cartrack::Multiplanar_Buffer_View> userspace_frames_cpu_views;
		userspace_frames_cpu_views.resize(num_buffers);

		for(auto buffer_index = 0; buffer_index < num_buffers; ++buffer_index)
		{
				userspace_frames.emplace_back(make_empty_frame());
				Frame_Impl& userspace_frame = userspace_frames[buffer_index];

				Cartrack::Multiplanar_Buffer_View& userspace_frame_view =
						userspace_frames_cpu_views[buffer_index];

				for(auto plane_index = 0; plane_index < num_planes; ++plane_index)
				{
						userspace_frame_view.emplace_back(std::span<Cartrack::Data_Type>(
								userspace_frame[plane_index].data(), userspace_frame[plane_index].size()));
				}
		}

		double average_capture_latency = 0;

		auto start_time = std::chrono::high_resolution_clock::now();

		for(int i = 0; i < num_frames; i += num_buffers)
		{
				backend->put_frame_data(userspace_frames_cpu_views);
				auto end_time = std::chrono::high_resolution_clock::now();

				std::chrono::duration<double, std::milli> elapsed_time = end_time - start_time;
				average_capture_latency += elapsed_time.count();
				std::cout << "Frame: " << i << "\ttiming for " << num_buffers
									<< " buffers: " << elapsed_time.count() << "\tms |\t"
									<< elapsed_time.count() / num_buffers << " for single frame." << std::endl;
				start_time= end_time;

				// for(int j = 0; j < num_buffers; ++j)
				// {
				// 		write_frame_to_disk(
				// 				userspace_frames_cpu_views[j], width, height, i + j + 1, false);
				// }
		}
		average_capture_latency /= num_frames;
		std::cout << "------------------------------------------------------------------------"
							<< std::endl;
		std::cout << "Average capture latency: " << average_capture_latency << " ms" << std::endl;
}

static void
mmap_capture(
		std::shared_ptr<Cartrack::V4L2_Backend> backend,
		uint num_frames = 1000)
{
		const auto num_buffers = backend->configuration().num_buffers;
		const int width				 = backend->get_width();
		const int height			 = backend->get_height();

		double average_capture_latency = 0;

		auto start_time = std::chrono::high_resolution_clock::now();
		for(int i = 0; i < num_frames; ++i)
		{
				auto frame			= backend->get_frame_data();
				auto end_time		= std::chrono::high_resolution_clock::now();
				std::chrono::duration<double, std::milli> elapsed_time = end_time - start_time;
				average_capture_latency += elapsed_time.count();
				std::cout << "Frame: " << i << "\ttiming with " << num_buffers
									<< " buffers: " << elapsed_time.count() << "\tms." << std::endl;
				start_time = end_time;

				// if(frame.empty())
				// {
				// 		continue;
				// }

				//write_frame_to_disk(frame, width, height, i + 1, true);
		}
		average_capture_latency /= num_frames;
		std::cout << "------------------------------------------------------------------------"
							<< std::endl;
		std::cout << "Average Frame Latency: " << average_capture_latency << " ms" << std::endl;
}

const static Cartrack::Stream_Configuration
get_test_setup(int camera_index=0,bool mmap=true)
{
		Cartrack::Stream_Configuration params;
		params.width				= 1920;
		params.height				= 1080;
		params.fps					= 30;
		params.num_buffers	= 4;
		params.buffering		= mmap ? Cartrack::Stream_Configuration::Buffering::Internal : Cartrack::Stream_Configuration::Buffering::USERPTR;
		params.pixel_format = Cartrack::Pixel_Format::NV12;
#ifdef ON_DEVICE
		params.v4l2.contiguous = false;
		params.device_index		 = camera_index; // 53, 62
#else
		params.v4l2.contiguous = true;
		params.device_index		 = 0;
#endif

#ifdef ON_DEVICE
		std::cout << "ON_DEVICE TEST SETUP LOADED" << std::endl;
#else
		std::cout << "DESKTOP TEST SETUP LOADED" << std::endl;
#endif
		std::cout <<"V4L2 Device Index: " << params.device_index << std::endl;
		std::cout << "Width: " << params.width << std::endl;
		std::cout << "Height: " << params.height << std::endl;
		std::cout << "FPS: " << params.fps << std::endl;
		std::cout << "Buffers: " << params.num_buffers << std::endl;
		std::cout << "Buffering: "
							<< (params.buffering == Cartrack::Stream_Configuration::Buffering::Internal
											? "Internal"
											: "External")
							<< std::endl;
		std::cout << "Pixel Format: " << static_cast<int>(params.pixel_format) << std::endl;

		return params;
}

auto
main(
		int argc,
		char* argv[]) -> int
{
		int camera_index = 0;
		if(argc > 1)
		{
				camera_index = std::atoi(argv[1]);
		}

		auto run_test = [](int cam, bool mmap)
		{
				const Cartrack::Stream_Configuration params = get_test_setup(cam);
				auto backend = std::make_shared<Cartrack::V4L2_Backend>(params);

				std::cout << "FPS: " << backend->get_fps() << std::endl;
				std::cout << "Width: " << backend->get_width() << std::endl;
				std::cout << "Height: " << backend->get_height() << std::endl;
				std::cout << "Automatic Exposure: " << backend->get_exposure() << std::endl;
				std::cout << "Automatic Exposure is_auto_exposure_auto_priority_enabled:"
									<< backend->is_auto_exposure_auto_priority_enabled() << std::endl;
				std::cout << "Automatic Exposure value: " << backend->get_auto_exposure_current_value()
									<< std::endl;

				std::cout << "------------------------------------------------------------------------"
									<< std::endl;

				if(mmap)
				{
						mmap_capture(backend, 100);
				}
				else
				{
						userptr_capture(backend, 100);
				}
		};


		run_test(camera_index,true);
		run_test(camera_index,false);

		return 0;
}
