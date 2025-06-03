#ifndef ISGURSOY_V4L2_HPP
#define ISGURSOY_V4L2_HPP

#include "Abstract_Capture_Backend.hpp"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <map>
#include <memory>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/mman.h>

#include <deque>
#include <iostream>
#include <array>
#include <cstring>
#include <numeric>
#include <compare>
#include <unistd.h>

namespace Cartrack
{

template <typename Floating_Point>
static bool
are_floats_equal(const Floating_Point a, const Floating_Point b)
{
		const std::partial_ordering result =
				std::abs(a - b) <=> std::numeric_limits<Floating_Point>::epsilon();
		return result == 0;
}

template <typename T>
void
zero_that(T& x)
{
		std::memset(&x, 0, sizeof(x));
}

static int
xioctl(int fd, int request, void* arg)
{
		int r = -1;
		do
		{
				r = ioctl(fd, request, arg);
		} while(-1 == r and EINTR == errno);

		return r;
}

class V4L2_Backend : public Capture_Backend
{
	public:
		explicit V4L2_Backend(const Stream_Configuration& params)
		{
				auto& _configuration_ = this->_configuration_;
				_configuration_				= params;
				if(pixel_formats_fourcc.find(_configuration_.pixel_format)
					 == pixel_formats_fourcc.end())
				{
						throw std::runtime_error("Pixel format not supported");
				}

				_buffer_plane_type_ = get_buffer_type_v4l2();
				_pixel_format_ = v4l2_fourcc(pixel_formats_fourcc.at(_configuration_.pixel_format)[0],
												pixel_formats_fourcc.at(_configuration_.pixel_format)[1],
												pixel_formats_fourcc.at(_configuration_.pixel_format)[2],
												pixel_formats_fourcc.at(_configuration_.pixel_format)[3]);
				this->_frame_order_ = 0;
				_device_dev_path_		= "/dev/video" + std::to_string(_configuration_.device_index);
				if(is_mjpeg())
				{
						_limit_range_ = false;
				}

				setup_device();

				setup_buffering();

				if(-1
					 == xioctl(
							 this->_device_file_descriptor_, VIDIOC_STREAMON, &this->_buffer_plane_type_))
				{
						throw std::runtime_error("VIDIOC_STREAMON");
				}
		}

		~V4L2_Backend() override
		{
				if(-1
					 == xioctl(_device_file_descriptor_, VIDIOC_STREAMOFF, &this->_buffer_plane_type_))
				{
						std::cerr << "VIDIOC_STREAMOFF failed" << std::endl;
				}

				if(get_memory_mapping_type_v4l2() == V4L2_MEMORY_MMAP
					 or get_memory_mapping_type_v4l2() == V4L2_MEMORY_DMABUF)
				{
						const int planes_count = this->num_planes();
						for(unsigned int i = 0; i < _num_buffers_; i++)
						{
								for(int j = 0; j < planes_count; ++j)
								{
										if(-1
											 == munmap(_mapped_buffers_[i][j].data(), _mapped_buffers_[i][j].size()))
										{
												std::cerr << "munmap failed" << std::endl;
										}
								}
						}
				}

				for(const auto& planes : _buffer_dma_fds_)
				{
						for(const auto& exbuf : planes)
						{
								if(-1 == close(exbuf.first))
								{
										std::cerr << "close failed" << std::endl;
								}
						}
				}

				if(-1 == close(_device_file_descriptor_))
				{
						std::cerr << "close failed" << std::endl;
				}
		}

	private:
		void setup_device()
		{
				auto& _configuration_ = this->_configuration_;
				struct stat st;

				if(stat(_device_dev_path_.c_str(), &st) == -1)
				{
						throw std::runtime_error("Cannot identify camera device: " + _device_dev_path_
																		 + " -> " + strerror(errno));
				}

				if(not S_ISCHR(st.st_mode))
				{
						throw std::runtime_error("Camera device is not a device: " + _device_dev_path_);
				}

				if(_device_file_descriptor_ = open(_device_dev_path_.c_str(), O_RDWR | O_NONBLOCK, 0);
					 -1 == _device_file_descriptor_)
				{
						throw std::runtime_error("Cannot open camera device " + _device_dev_path_ + " -> "
																		 + strerror(errno));
				}

				v4l2_capability v4l2_environment;

				if(-1 == xioctl(this->_device_file_descriptor_, VIDIOC_QUERYCAP, &v4l2_environment))
				{
						if(EINVAL == errno)
						{
								throw std::runtime_error("VIDIOC_QUERYCAP: " + _device_dev_path_ + " -> "
																				 + strerror(errno));
						}
						else
						{
								throw std::runtime_error("VIDIOC_QUERYCAP: " + _device_dev_path_);
						}
				}

				if(!(v4l2_environment.capabilities & V4L2_CAP_VIDEO_CAPTURE)
					 and !(v4l2_environment.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE))
				{
						throw std::runtime_error(
								"Camera device is not a video capture device that is able to "
								"V4L2_CAP_VIDEO_CAPTURE or V4L2_CAP_VIDEO_CAPTURE_MPLANE");
				}

				if(!(v4l2_environment.capabilities & V4L2_CAP_STREAMING))
				{
						throw std::runtime_error("Camera device does not support streaming i/o.");
				}

				struct v4l2_cropcap cropcap;
				zero_that(cropcap);

				cropcap.type = _buffer_plane_type_;

				if(0 == xioctl(_device_file_descriptor_, VIDIOC_CROPCAP, &cropcap))
				{
						const auto sum = std::accumulate(_configuration_.v4l2.crop_rect.begin(),
																						 _configuration_.v4l2.crop_rect.end(),
																						 0);
						if(sum)
						{
								struct v4l2_crop crop;
								zero_that(crop);
								crop.type = _buffer_plane_type_;
								crop.c		= v4l2_rect{(int) _configuration_.v4l2.crop_rect[0],
																		(int) _configuration_.v4l2.crop_rect[1],
																		(unsigned int) _configuration_.v4l2.crop_rect[2],
																		(unsigned int) _configuration_.v4l2.crop_rect[3]};

								if(-1 == xioctl(_device_file_descriptor_, VIDIOC_S_CROP, &crop))
								{
										switch(errno)
										{
												case EINVAL:
														std::cerr << "Cropping not supported." << std::endl;
														break;
												default:
														/* Errors ignored. */
														std::cerr << "VIDIOC_S_CROP: " << strerror(errno) << std::endl;
														break;
										}
								}
								else
								{
										std::cout << "cropcap.type: " << cropcap.type << std::endl;
										std::cout << "cropcap.bounds.left: " << cropcap.bounds.left << std::endl;
										std::cout << "cropcap.bounds.top: " << cropcap.bounds.top << std::endl;
										std::cout << "cropcap.bounds.width: " << cropcap.bounds.width << std::endl;
										std::cout << "cropcap.bounds.height: " << cropcap.bounds.height
															<< std::endl;
										std::cout << "cropcap.defrect.left: " << cropcap.defrect.left << std::endl;
										std::cout << "cropcap.defrect.top: " << cropcap.defrect.top << std::endl;
										std::cout << "cropcap.defrect.width: " << cropcap.defrect.width
															<< std::endl;
										std::cout << "cropcap.defrect.height: " << cropcap.defrect.height
															<< std::endl;
										std::cout << "cropcap.pixelaspect.numerator : "
															<< cropcap.pixelaspect.numerator << std::endl;
										std::cout << "cropcap.pixelaspect.denominator : "
															<< cropcap.pixelaspect.denominator << std::endl;
										std::cout << std::endl;
								}
						}
				}

				zero_that(_v4l2_capture_format_);

				_v4l2_capture_format_.type = this->_buffer_plane_type_;
				if(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == this->_buffer_plane_type_)
				{
						_v4l2_capture_format_.fmt.pix_mp.width			 = _configuration_.width;
						_v4l2_capture_format_.fmt.pix_mp.height			 = _configuration_.height;
						_v4l2_capture_format_.fmt.pix_mp.pixelformat = this->_pixel_format_;
						_v4l2_capture_format_.fmt.pix_mp.field =
								is_mjpeg() ? V4L2_FIELD_NONE : V4L2_FIELD_INTERLACED;

						if(_limit_range_)
						{
								_v4l2_capture_format_.fmt.pix_mp.quantization = V4L2_QUANTIZATION_LIM_RANGE;
						}
						else
						{
								_v4l2_capture_format_.fmt.pix_mp.quantization = V4L2_QUANTIZATION_FULL_RANGE;
						}
				}
				else
				{
						_v4l2_capture_format_.fmt.pix.width				= _configuration_.width;
						_v4l2_capture_format_.fmt.pix.height			= _configuration_.height;
						_v4l2_capture_format_.fmt.pix.pixelformat = this->_pixel_format_;
						_v4l2_capture_format_.fmt.pix.field =
								is_mjpeg() ? V4L2_FIELD_NONE : V4L2_FIELD_INTERLACED;

						if(_limit_range_)
						{
								_v4l2_capture_format_.fmt.pix.quantization = V4L2_QUANTIZATION_LIM_RANGE;
						}
						else
						{
								_v4l2_capture_format_.fmt.pix.quantization = V4L2_QUANTIZATION_FULL_RANGE;
						}
				}

				/**
     * This, VIDIOC_S_FMT, is one time setting, so you can NOT set twice.
     * You can't change the format after this point.
     * You need to free the buffers before, using VIDIOC_REQBUFS with a buffer
     * count of 0. S stands for SET.
     */

				if(-1 == xioctl(this->_device_file_descriptor_, VIDIOC_S_FMT, &_v4l2_capture_format_))
				{
						/* Note VIDIOC_S_FMT may change width and height. */
						throw std::runtime_error("VIDIOC_S_FMT: " + std::string(strerror(errno)));
				}

				std::cout << "Fps is set to: " << set_fps(_configuration_.fps) << std::endl;
				set_auto_exposure_mode(
						/*V4L2_EXPOSURE_MANUAL ,*/ V4L2_EXPOSURE_APERTURE_PRIORITY);
				enable_auto_exposure_auto_priority_mode(false);
		}

		void setup_buffering()
		{
				auto& _configuration_ = this->_configuration_;
				v4l2_requestbuffers req;
				zero_that(req);
				req.count				= _configuration_.num_buffers;
				req.type				= this->_buffer_plane_type_;
				req.memory			= get_memory_mapping_type_v4l2();
				req.reserved[0] = 0;

				if(-1 == xioctl(this->_device_file_descriptor_, VIDIOC_REQBUFS, &req))
				{
						if(EINVAL == errno)
						{
								throw std::runtime_error(std::string{"%s: %s does not support "
																										 "user pointer mapping "
																										 + this->_device_dev_path_});
						}
						else
						{
								throw std::runtime_error("VIDIOC_REQBUFS");
						}
				}

				const int planes_count = this->num_planes();

				if(req.count < 1)
				{
						throw std::runtime_error(
								std::string{"Insufficient buffer memory for "} + _device_dev_path_
								+ " | increasing num_max_internal_buffers which is currently "
								+ std::to_string(_configuration_.num_buffers) + " may work.");
				}

				std::cerr << "Device fd is: " << this->_device_file_descriptor_ << std::endl;
				std::cout << "Num buffers to be used: " << req.count << std::endl;

				if(get_memory_mapping_type_v4l2() == V4L2_MEMORY_MMAP
					 or get_memory_mapping_type_v4l2() == V4L2_MEMORY_DMABUF)
				{
						_num_buffers_ = req.count;
						_buffer_dma_fds_.resize(_num_buffers_);
						this->_mapped_buffers_.resize(_num_buffers_);

						for(auto buffer_index = 0; buffer_index < _num_buffers_; ++buffer_index)
						{
								v4l2_buffer buf;
								zero_that(buf);
								v4l2_plane planes[planes_count];
								memset(planes, 0, planes_count * sizeof(v4l2_plane));

								buf.type			= this->_buffer_plane_type_;
								buf.memory		= get_memory_mapping_type_v4l2();
								buf.index			= buffer_index;
								buf.reserved2 = 0;
								buf.reserved	= 0;

								if(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == this->_buffer_plane_type_)
								{
										buf.m.planes = planes;
										buf.length	 = planes_count;
								}
								else
								{
										buf.length = _v4l2_capture_format_.fmt.pix.sizeimage;
								}

								if(-1 == xioctl(this->_device_file_descriptor_, VIDIOC_QUERYBUF, &buf))
								{
										throw std::runtime_error("VIDIOC_QUERYBUF");
								}

								_mapped_buffers_[buffer_index].resize(planes_count);

								v4l2_exportbuffer expbuf;

								bool dmabuf = true;

								if(dmabuf)
								{
										zero_that(expbuf);
										expbuf.type	 = this->_buffer_plane_type_;
										expbuf.index = buffer_index;
										expbuf.flags = O_CLOEXEC | O_RDWR;
								}

								if(V4L2_BUF_TYPE_VIDEO_CAPTURE == this->_buffer_plane_type_)
								{
										if(dmabuf)
										{
												expbuf.plane = 0;

												const auto err =
														ioctl(this->_device_file_descriptor_, VIDIOC_EXPBUF, &expbuf);
												if(err == -1)
												{
														// close(this->_device_file_descriptor_);
														// throw std::runtime_error("VIDIOC_EXPBUF: "
														// 												 + std::string{strerror(errno)});
														std::cout << "VIDIOC_EXPBUF: " << std::string{strerror(errno)}
																			<< std::endl;
														std::cerr << "dma buf is not available in this environment."
																			<< std::endl;
														dmabuf = false;
												}
												else
												{
														std::cout << "DMABUF FD for buf: " << expbuf.index << " is "
																			<< expbuf.fd << std::endl;
														_buffer_dma_fds_[buffer_index].emplace_back(expbuf.fd, buf.length);
												}
										}

										_mapped_buffers_[buffer_index][0] = std::span(
												(Data_Type*) mmap(nullptr /* start anywhere */,
																					buf.length,
																					PROT_READ | PROT_WRITE /* required */,
																					MAP_SHARED /* recommended */,
																					dmabuf ? expbuf.fd : this->_device_file_descriptor_,
																					dmabuf ? 0 : buf.m.offset),
												buf.length);

										if((void*) -1 == _mapped_buffers_[buffer_index][0].data())
										{
												std::cerr << "----Error----" << std::endl;

												long page_size			 = sysconf(_SC_PAGESIZE);
												off_t offset				 = buf.m.offset;
												off_t aligned_offset = offset - (offset % page_size);

												std::cerr << "Buffer: " << expbuf.index << std::endl;
												std::cerr << "Page size: " << page_size << std::endl;
												std::cerr << "Aligned offset: " << aligned_offset << std::endl;
												std::cerr << buf.length << " is length | "
																	<< (dmabuf ? expbuf.fd : this->_device_file_descriptor_)
																	<< " is fd | " << buf.m.offset << " is offset." << std::endl;
												throw std::runtime_error("mmap: " + std::string{strerror(errno)});
										}
								}
								else
								{
										for(auto plane_index = 0; plane_index < planes_count; ++plane_index)
										{
												if(dmabuf)
												{
														expbuf.plane = plane_index;
														if(ioctl(this->_device_file_descriptor_, VIDIOC_EXPBUF, &expbuf)
															 == -1)
														{
																// close(this->_device_file_descriptor_);
																// throw std::runtime_error("VIDIOC_EXPBUF: "
																// 													 + std::string{strerror(errno)});
																std::cout << "VIDIOC_EXPBUF: " << std::string{strerror(errno)}
																					<< std::endl;
																std::cerr << "dma buf is not available in this environment."
																					<< std::endl;
																dmabuf = false;
														}
														else
														{
																std::cout << "DMABUF FD for buf: " << expbuf.index << " is "
																					<< expbuf.fd << std::endl;
																_buffer_dma_fds_[buffer_index].emplace_back(expbuf.fd,
																																						buf.length);
														}
												}

												_mapped_buffers_[buffer_index][plane_index] =
														std::span((Data_Type*) mmap(
																					nullptr /* start anywhere */,
																					buf.m.planes[plane_index].length,
																					PROT_READ | PROT_WRITE /* required */,
																					MAP_SHARED /* recommended */,
																					dmabuf ? expbuf.fd : this->_device_file_descriptor_,
																					dmabuf ? 0 : buf.m.planes[plane_index].m.mem_offset),
																			buf.m.planes[plane_index].length);

												if((void*) -1 == _mapped_buffers_[buffer_index][plane_index].data())
												{
														std::cerr << "----Error----" << std::endl;

														long page_size			 = sysconf(_SC_PAGESIZE);
														off_t offset				 = buf.m.planes[plane_index].m.mem_offset;
														off_t aligned_offset = offset - (offset % page_size);

														std::cerr << "Buffer: " << expbuf.index << std::endl;
														std::cerr << "Page size: " << page_size << std::endl;
														std::cerr << "Aligned offset: " << aligned_offset << std::endl;
														std::cerr << buf.m.planes[plane_index].length << " is length | "
																			<< (dmabuf ? expbuf.fd : this->_device_file_descriptor_)
																			<< " is fd | " << buf.m.planes[plane_index].m.mem_offset
																			<< " is offset" << std::endl;
														throw std::runtime_error("mmap: " + std::string{strerror(errno)});
												}
										}
								}
						}

						for(unsigned int i = 0; i < this->_num_buffers_; ++i)
						{
								v4l2_buffer buf;
								zero_that(buf);
								buf.type	 = this->_buffer_plane_type_;
								buf.memory = get_memory_mapping_type_v4l2();
								buf.index	 = i;

								v4l2_plane planes[planes_count];
								if(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == this->_buffer_plane_type_)
								{
										memset(planes, 0, planes_count * sizeof(v4l2_plane));
										buf.m.planes = planes;
										buf.length	 = planes_count;
								}

								if(-1 == xioctl(this->_device_file_descriptor_, VIDIOC_QBUF, &buf))
								{
										const int err = errno;
										throw std::runtime_error("VIDIOC_QBUF setup_buffering: "
																						 + std::string(strerror(err)));
								}
						}
				}
				else if(get_memory_mapping_type_v4l2() == V4L2_MEMORY_USERPTR)
				{
						// VIDIOC_QUERYBUF is not neded for user pointer mapping

						_num_buffers_ = req.count;

						_allocated_buffers_.resize(_num_buffers_);
						for(auto buffer_index = 0; buffer_index < _num_buffers_; ++buffer_index)
						{
								_allocated_buffers_[buffer_index].resize(planes_count);

								if(V4L2_BUF_TYPE_VIDEO_CAPTURE == this->_buffer_plane_type_)
								{
										const auto image_size = _v4l2_capture_format_.fmt.pix.sizeimage;
										_allocated_buffers_[buffer_index][0].resize(image_size);
								}
								else
								{
										for(int j = 0; j < planes_count; ++j)
										{
												const auto image_size =
														_v4l2_capture_format_.fmt.pix_mp.plane_fmt[j].sizeimage;
												_allocated_buffers_[buffer_index][j].resize(image_size);
										}
								}
						}
				}
		}

		[[nodiscard]] v4l2_memory get_memory_mapping_type_v4l2() const
		{
				const auto& _configuration_ = this->_configuration_;
				return _configuration_.buffering == Stream_Configuration::Buffering::Internal
									 ? //
									 V4L2_MEMORY_MMAP
									 // V4L2_MEMORY_DMABUF
									 : V4L2_MEMORY_USERPTR;
		}

		[[nodiscard]] v4l2_buf_type get_buffer_type_v4l2() const
		{
				const auto& _configuration_ = this->_configuration_;
				return (_configuration_.v4l2.contiguous) ? V4L2_BUF_TYPE_VIDEO_CAPTURE
																								 : V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		}

		[[nodiscard]] constexpr size_t num_planes() const override
		{
				return _v4l2_capture_format_.type == V4L2_BUF_TYPE_VIDEO_CAPTURE
									 ? 1
									 : _v4l2_capture_format_.fmt.pix_mp.num_planes;
		}

		[[nodiscard]] bool is_mjpeg() const
		{
				return this->_pixel_format_ == V4L2_PIX_FMT_MJPEG;
		}

		[[nodiscard]] bool try_device() const
		{
				fd_set fds;
				struct timeval tv;
				int r = 0;

				FD_ZERO(&fds);
				FD_SET(_device_file_descriptor_, &fds);

				/* Timeout. */
				tv.tv_sec	 = 0;
				tv.tv_usec = _timeout_in_milli * 1000;

				r = select(_device_file_descriptor_ + 1, &fds, NULL, NULL, &tv);

				if(-1 == r)
				{
						if(EINTR == errno)
						{
								std::cerr << "select() interrupted" << std::endl;
								return false;
						}
				}

				if(0 == r)
				{
						std::cerr << "select() timeout" << std::endl;
						return false;
				}

				return true;
		}

	public:
		[[nodiscard]] Multiplanar_Buffer_View get_frame_data() override
		{
				auto& _configuration_ = this->_configuration_;
				auto& _frame_order_		= this->_frame_order_;
				if(get_memory_mapping_type_v4l2() == V4L2_MEMORY_USERPTR)
				{
						const int planes_count = this->num_planes();

						std::vector<Multiplanar_Buffer_View> per_buffer_planes;
						per_buffer_planes.resize(_num_buffers_);

						for(int buffer_index = 0; buffer_index < _num_buffers_; ++buffer_index)
						{
								per_buffer_planes[buffer_index].resize(planes_count);
								for(auto plane_index = 0; plane_index < planes_count; ++plane_index)
								{
										per_buffer_planes[buffer_index][plane_index] = std::span<Data_Type>(
												(Data_Type*) _allocated_buffers_[buffer_index][plane_index].data(),
												_allocated_buffers_[buffer_index][plane_index].size());
								}
						}
						std::vector<std::vector<size_t>> bytes_used_per_buffer =
								put_frame_data(per_buffer_planes);
						if(bytes_used_per_buffer.empty())
						{
								return {};
						}
						if(bytes_used_per_buffer.front().empty())
						{
								return {};
						}

						Multiplanar_Buffer_View planes_to_return;

						if(_configuration_.v4l2.buffer_usage_policy
							 == Stream_Configuration::V4L2::Internal_Buffering_Strategy::Oldest)
						{
								const auto& buffer_to_use = per_buffer_planes[0];
								const auto bytes_used			= bytes_used_per_buffer[0];

								for(auto plane_index = 0; plane_index < planes_count; ++plane_index)
								{
										planes_to_return.emplace_back(buffer_to_use[plane_index].data(),
																									bytes_used[plane_index]);
								}
						}
						else
						{
								const auto& buffer_to_use = per_buffer_planes.back();
								const auto& bytes_used		= bytes_used_per_buffer.back();

								for(auto plane_index = 0; plane_index < planes_count; ++plane_index)
								{
										planes_to_return.emplace_back(buffer_to_use[plane_index].data(),
																									bytes_used[plane_index]);
								}
						}
						return planes_to_return;
				}

				const int planes_count = this->num_planes();

				Multiplanar_Buffer_View planes_to_return;

				auto take_span = [this, &planes_count](const v4l2_buffer& buf,
														 Multiplanar_Buffer_View& collected_planes)
				{
						if(get_buffer_type_v4l2() == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
						{
								for(auto plane_index = 0; plane_index < planes_count; ++plane_index)
								{
										collected_planes.emplace_back(
												std::assume_aligned<Alignment_Size>(
														_mapped_buffers_[buf.index][plane_index].data()),
												buf.m.planes[plane_index].bytesused);
								}
						}
						else if(get_buffer_type_v4l2() == V4L2_BUF_TYPE_VIDEO_CAPTURE)
						{
								collected_planes.emplace_back(
										std::assume_aligned<Alignment_Size>(_mapped_buffers_[buf.index][0].data()),
										buf.bytesused);
						}
				};

				auto queue_buf = [this](v4l2_buffer& buf)
				{
						if(-1 == xioctl(_device_file_descriptor_, VIDIOC_QBUF, &buf))
						{
								const int err = errno; // Get the error number
								std::cerr << "VIDIOC_QBUF failed in frame grabbing" << err << ": "
													<< strerror(err) << std::endl;
								return false;
						}
						return true;
				};

				auto deque_buf = [this](v4l2_buffer& buf)
				{
						if(-1 == xioctl(_device_file_descriptor_, VIDIOC_DQBUF, &buf))
						{
								switch(errno)
								{
										case EAGAIN:
												return false;

										case EIO:
												/* Could ignore EIO, see spec. */
												/* fall through */
												break;
										default:
												const int err = errno; // Get the error number
												std::cerr << "VIDIOC_DQBUF failed in frame grabbing" << err << ": "
																	<< strerror(err) << std::endl;
												return false;
								}
						}

						if(buf.index >= _num_buffers_)
						{
								return false;
						}

						return true;
				};

				auto instantiate_buf = [this]() -> v4l2_buffer
				{
						v4l2_buffer buf;
						zero_that(buf);

						buf.type	 = get_buffer_type_v4l2();
						buf.memory = get_memory_mapping_type_v4l2();

						v4l2_plane planes[this->num_planes()];
						if(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == this->_buffer_plane_type_)
						{
								memset(planes, 0, this->num_planes() * sizeof(v4l2_plane));
								buf.m.planes = planes;
								buf.length	 = this->num_planes();
						}

						return buf;
				};

				while(not _buffer_of_buffers.empty())
				{
						auto buf = _buffer_of_buffers.front();
						queue_buf(buf);
						_buffer_of_buffers.pop_front();
				}

				if(not try_device())
				{
						std::cout << "You are requesting frame faster than the fps you set: "
											<< this->get_fps() << ", will return empty frame." << std::endl;
						return {};
				}

				if(_configuration_.v4l2.buffer_usage_policy
					 == Stream_Configuration::V4L2::Internal_Buffering_Strategy::Oldest)
				{
						auto buf = instantiate_buf();

						++_frame_order_;

						if(deque_buf(buf))
						{
								take_span(buf, planes_to_return);
						}
						_buffer_of_buffers.push_back(buf);
				}
				else if(_configuration_.v4l2.buffer_usage_policy
								== Stream_Configuration::V4L2::Internal_Buffering_Strategy::Only_Newest)
				{
						static const unsigned int dummy_buffer_index =
								std::numeric_limits<unsigned int>::max() - 1;

						std::map<long, std::pair<Multiplanar_Buffer_View, v4l2_buffer>> ordered_buffers;
						for(auto buffer_order = 0; buffer_order < this->_num_buffers_; ++buffer_order)
						{
								auto buf	= instantiate_buf();
								buf.index = dummy_buffer_index;

								++_frame_order_;

								if(deque_buf(buf))
								{
										if(buf.index == dummy_buffer_index)
										{
												continue;
										}

										ordered_buffers[buf.timestamp.tv_usec].second = buf;
										take_span(buf, ordered_buffers[buf.timestamp.tv_usec].first);
								}
						}

						if(ordered_buffers.empty())
						{
								return {};
						}

						if(ordered_buffers.size() == 1)
						{
								planes_to_return = ordered_buffers.begin()->second.first;
								_buffer_of_buffers.push_back(ordered_buffers.begin()->second.second);
								return planes_to_return;
						}

						planes_to_return = ordered_buffers.rbegin()->second.first;
						_buffer_of_buffers.push_back(ordered_buffers.rbegin()->second.second);

						for(auto map_iterator = ordered_buffers.begin();
								map_iterator not_eq ordered_buffers.end();
								++map_iterator)
						{
								if(std::next(map_iterator) not_eq ordered_buffers.end())
								{
										queue_buf(map_iterator->second.second);
								}
						}
				}

				return planes_to_return;
		}

		[[nodiscard]] std::vector<std::vector<size_t>> put_frame_data(
				std::vector<Multiplanar_Buffer_View>& userspace_frames) override
		{
				auto& _frame_order_ = this->_frame_order_;

				std::vector<std::vector<size_t>> sizes;
				sizes.resize(userspace_frames.size());
				const int planes_count = this->num_planes();
				for(auto& bytes_used : sizes)
				{
						bytes_used.resize(planes_count);
				}

				if(get_memory_mapping_type_v4l2() == V4L2_MEMORY_MMAP
					 or get_memory_mapping_type_v4l2() == V4L2_MEMORY_DMABUF)
				{
						for(int userspace_frame_index = 0; userspace_frame_index < userspace_frames.size();
								++userspace_frame_index)
						{
								auto& userspace_frame							= userspace_frames[userspace_frame_index];
								const auto mmap_frame_buffer_view = get_frame_data();

								if(mmap_frame_buffer_view.empty())
								{
										continue;
								}

								if(userspace_frame.size() < mmap_frame_buffer_view.size())
								{
										continue;
								}

								for(auto plane_index = 0; plane_index < planes_count; ++plane_index)
								{
										try{
										std::copy(mmap_frame_buffer_view[plane_index].begin(),
															mmap_frame_buffer_view[plane_index].end(),
															userspace_frame[plane_index].begin());
										sizes[userspace_frame_index][plane_index] =
												mmap_frame_buffer_view[plane_index].size();
										}
										catch(std::exception& e)
										{
												std::cout << "Exception: " << e.what() << std::endl;
												sizes[userspace_frame_index][plane_index] = 0;
										}
								}
						}
						return sizes;
				}

				ushort num_queued_buffers = 0;
				for(auto userspace_frame_index = 0; userspace_frame_index < userspace_frames.size();
						++userspace_frame_index)
				{
						auto& userspace_frame = userspace_frames[userspace_frame_index];
						v4l2_buffer buf;
						zero_that(buf);
						buf.type	 = get_buffer_type_v4l2();
						buf.memory = get_memory_mapping_type_v4l2();
						buf.index	 = userspace_frame_index;
						buf.flags			= 0;
						buf.reserved2 = 0;
						buf.reserved	= 0;

						v4l2_plane planes[planes_count];
						if(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == this->_buffer_plane_type_)
						{
								memset(planes, 0, planes_count * sizeof(v4l2_plane));
								buf.length = planes_count;

								for(auto plane_index = 0; plane_index < buf.length; ++plane_index)
								{
										planes[plane_index].m.userptr = ulong(userspace_frame[plane_index].data());
										planes[plane_index].length =
												_v4l2_capture_format_.fmt.pix_mp.plane_fmt[plane_index].sizeimage;
										planes[plane_index].data_offset	 = 0;
										planes[plane_index].m.mem_offset = 0;
								}

								buf.m.planes = planes;
						}
						else
						{
								buf.m.userptr = ulong(userspace_frame[0].data());
								buf.length		= _v4l2_capture_format_.fmt.pix.sizeimage;
						}

						if(-1 == xioctl(this->_device_file_descriptor_, VIDIOC_QBUF, &buf))
						{
								const int err = errno; // Get the error number
								std::cerr << "VIDIOC_QBUF error " << err << ": " << strerror(err) << std::endl;

								if(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == this->_buffer_plane_type_)
								{
										for(auto plane_index = 0; plane_index < buf.length; ++plane_index)
										{
												sizes[userspace_frame_index][plane_index] = 0;
										}
								}
								else
								{
										sizes[userspace_frame_index][0] = 0;
								}
						}
						else
						{
								++num_queued_buffers;
						}

						++_frame_order_;
				}

				if(not try_device())
				{
						std::cout << "You are requesting frame faster than the fps you set: "
											<< this->get_fps() << ", may return empty frame." << std::endl;
				}

				for(int num_buffer_requests = 0; num_buffer_requests < num_queued_buffers;
						++num_buffer_requests)
				{
						v4l2_buffer buf;
						zero_that(buf);
						buf.type	 = get_buffer_type_v4l2();
						buf.memory = get_memory_mapping_type_v4l2();

						v4l2_plane planes[planes_count];
						if(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == this->_buffer_plane_type_)
						{
								memset(planes, 0, planes_count * sizeof(v4l2_plane));
								buf.length	 = planes_count;
								buf.m.planes = planes;
						}

						if(-1 == xioctl(_device_file_descriptor_, VIDIOC_DQBUF, &buf))
						{
								switch(errno)
								{
										case EAGAIN:
												--num_buffer_requests;
												try_device();
												break;
										case EIO:
												/* Could ignore EIO, see spec. */
												/* fall through */
												break;
										default:
												std::cerr << "VIDIOC_DQBUF" << std::endl;
												break;
								}
						}
						else
						{
								if(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == this->_buffer_plane_type_)
								{
										for(auto plane_index = 0; plane_index < buf.length; ++plane_index)
										{
												sizes[buf.index][plane_index] = buf.m.planes[plane_index].bytesused;
										}
								}
								else
								{
										sizes[buf.index][0] = buf.bytesused;
								}
						}
				}

				return sizes;
		}

		[[nodiscard]] Pixel_Format get_pixel_format() const override
		{
				auto& _configuration_ = this->_configuration_;
				return _configuration_.pixel_format;
		}

		bool set_zoom(int value) override
		{
				struct v4l2_control ctrl = {0};
				ctrl.id									 = V4L2_CID_ZOOM_ABSOLUTE;
				ctrl.value							 = value;
				if(-1 == xioctl(_device_file_descriptor_, VIDIOC_S_CTRL, &ctrl))
				{
						std::cerr << "error setting V4L2_CID_ZOOM_ABSOLUTE" << std::endl;
						return false;
				}
				return true;
		}

		[[nodiscard]] int get_zoom() const override
		{
				struct v4l2_control ctrl = {0};
				ctrl.id									 = V4L2_CID_ZOOM_ABSOLUTE;
				if(-1 == xioctl(_device_file_descriptor_, VIDIOC_G_CTRL, &ctrl))
				{
						std::cerr << "Error getting V4L2_CID_ZOOM_ABSOLUTE" << std::endl;
				}
				return ctrl.value;
		}

		bool set_focus(int value) override
		{
				struct v4l2_control ctrl = {0};
				ctrl.id									 = V4L2_CID_FOCUS_ABSOLUTE;
				ctrl.value							 = value;
				if(-1 == xioctl(_device_file_descriptor_, VIDIOC_S_CTRL, &ctrl))
				{
						std::cerr << "error setting V4L2_CID_FOCUS_ABSOLUTE" << std::endl;
						return false;
				}
				return true;
		}

		[[nodiscard]] int get_focus() const override
		{
				struct v4l2_control ctrl = {0};
				ctrl.id									 = V4L2_CID_FOCUS_ABSOLUTE;
				if(-1 == xioctl(_device_file_descriptor_, VIDIOC_G_CTRL, &ctrl))
				{
						std::cerr << "error getting V4L2_CID_FOCUS_ABSOLUTE" << std::endl;
				}
				return ctrl.value;
		}

		bool set_sharpness(int value) override
		{
				struct v4l2_control ctrl = {0};
				ctrl.id									 = V4L2_CID_SHARPNESS;
				ctrl.value							 = value;
				if(-1 == xioctl(_device_file_descriptor_, VIDIOC_S_CTRL, &ctrl))
				{
						std::cerr << "error setting V4L2_CID_SHARPNESS" << std::endl;
						return false;
				}
				return true;
		}

		[[nodiscard]] int get_sharpness() const override
		{
				struct v4l2_control ctrl = {0};
				ctrl.id									 = V4L2_CID_SHARPNESS;
				if(-1 == xioctl(_device_file_descriptor_, VIDIOC_G_CTRL, &ctrl))
				{
						std::cerr << "error getting V4L2_CID_SHARPNESS" << std::endl;
				}
				return ctrl.value;
		}

		bool set_auto_focus(bool value) override
		{
				struct v4l2_control ctrl = {0};
				ctrl.id									 = V4L2_CID_FOCUS_AUTO;
				ctrl.value							 = value;
				if(-1 == xioctl(_device_file_descriptor_, VIDIOC_S_CTRL, &ctrl))
				{
						std::cerr << "error setting V4L2_CID_FOCUS_AUTO" << std::endl;
						return false;
				}
				return true;
		}

		[[nodiscard]] bool get_auto_focus() const override
		{
				struct v4l2_control ctrl = {0};
				ctrl.id									 = V4L2_CID_FOCUS_AUTO;
				if(-1 == xioctl(_device_file_descriptor_, VIDIOC_G_CTRL, &ctrl))
				{
						std::cerr << "error getting V4L2_CID_FOCUS_AUTO" << std::endl;
				}
				return ctrl.value;
		}

		bool set_brightness(int value) override
		{
				struct v4l2_control ctrl = {0};
				ctrl.id									 = V4L2_CID_BRIGHTNESS;
				ctrl.value							 = value;
				if(-1 == xioctl(_device_file_descriptor_, VIDIOC_S_CTRL, &ctrl))
				{
						std::cerr << "error setting V4L2_CID_BRIGHTNESS" << std::endl;
						return false;
				}
				return true;
		}

		[[nodiscard]] int get_brightness() const override
		{
				struct v4l2_control ctrl = {0};
				ctrl.id									 = V4L2_CID_BRIGHTNESS;
				if(-1 == xioctl(_device_file_descriptor_, VIDIOC_G_CTRL, &ctrl))
				{
						std::cerr << "error getting V4L2_CID_BRIGHTNESS" << std::endl;
				}
				return ctrl.value;
		}

		bool set_contrast(int value) override
		{
				struct v4l2_control ctrl = {0};
				ctrl.id									 = V4L2_CID_CONTRAST;
				ctrl.value							 = value;
				if(-1 == xioctl(_device_file_descriptor_, VIDIOC_S_CTRL, &ctrl))
				{
						std::cerr << "error setting V4L2_CID_CONTRAST" << std::endl;
						return false;
				}
				return true;
		}

		[[nodiscard]] int get_contrast() const override
		{
				struct v4l2_control ctrl = {0};
				ctrl.id									 = V4L2_CID_CONTRAST;
				if(-1 == xioctl(_device_file_descriptor_, VIDIOC_G_CTRL, &ctrl))
				{
						std::cerr << "error getting V4L2_CID_CONTRAST" << std::endl;
				}
				return ctrl.value;
		}

		bool set_saturation(int value) override
		{
				struct v4l2_control ctrl = {0};
				ctrl.id									 = V4L2_CID_SATURATION;
				ctrl.value							 = value;
				if(-1 == xioctl(_device_file_descriptor_, VIDIOC_S_CTRL, &ctrl))
				{
						std::cerr << "error setting V4L2_CID_SATURATION" << std::endl;
						return false;
				}
				return true;
		}

		[[nodiscard]] int get_saturation() const override
		{
				struct v4l2_control ctrl = {0};
				ctrl.id									 = V4L2_CID_SATURATION;
				if(-1 == xioctl(_device_file_descriptor_, VIDIOC_G_CTRL, &ctrl))
				{
						std::cerr << "error getting V4L2_CID_SATURATION" << std::endl;
				}
				return ctrl.value;
		}

		bool set_hue(int value) override
		{
				struct v4l2_control ctrl = {0};
				ctrl.id									 = V4L2_CID_HUE;
				ctrl.value							 = value;
				if(-1 == xioctl(_device_file_descriptor_, VIDIOC_S_CTRL, &ctrl))
				{
						std::cerr << "error setting V4L2_CID_HUE" << std::endl;
						return false;
				}
				return true;
		}

		[[nodiscard]] int get_hue() const override
		{
				struct v4l2_control ctrl = {0};
				ctrl.id									 = V4L2_CID_HUE;
				if(-1 == xioctl(_device_file_descriptor_, VIDIOC_G_CTRL, &ctrl))
				{
						std::cerr << "error getting V4L2_CID_HUE" << std::endl;
				}
				return ctrl.value;
		}

		bool set_gain(int value) override
		{
				struct v4l2_control ctrl = {0};
				ctrl.id									 = V4L2_CID_GAIN;
				ctrl.value							 = value;
				if(-1 == xioctl(_device_file_descriptor_, VIDIOC_S_CTRL, &ctrl))
				{
						std::cerr << "error setting V4L2_CID_GAIN" << std::endl;
						return false;
				}
				return true;
		}

		[[nodiscard]] int get_gain() const override
		{
				struct v4l2_control ctrl = {0};
				ctrl.id									 = V4L2_CID_GAIN;
				if(-1 == xioctl(_device_file_descriptor_, VIDIOC_G_CTRL, &ctrl))
				{
						std::cerr << "error getting V4L2_CID_GAIN" << std::endl;
				}
				return ctrl.value;
		}

		bool set_exposure(int value) override
		{
				struct v4l2_control ctrl = {0};
				ctrl.id									 = V4L2_CID_EXPOSURE_ABSOLUTE;
				ctrl.value							 = value;
				if(-1 == xioctl(_device_file_descriptor_, VIDIOC_S_CTRL, &ctrl))
				{
						std::cerr << "error setting V4L2_CID_EXPOSURE_ABSOLUTE" << std::endl;
						return false;
				}
				return true;
		}

		[[nodiscard]] int get_exposure() const override
		{
				struct v4l2_control ctrl = {0};
				ctrl.id									 = V4L2_CID_EXPOSURE_ABSOLUTE;
				if(-1 == xioctl(_device_file_descriptor_, VIDIOC_G_CTRL, &ctrl))
				{
						std::cerr << "error getting V4L2_CID_EXPOSURE_ABSOLUTE" << std::endl;
				}
				return ctrl.value;
		}

		bool set_white_balance_temperature(int value) override
		{
				struct v4l2_control ctrl = {0};
				ctrl.id									 = V4L2_CID_WHITE_BALANCE_TEMPERATURE;
				ctrl.value							 = value;
				if(-1 == xioctl(_device_file_descriptor_, VIDIOC_S_CTRL, &ctrl))
				{
						std::cerr << "error setting V4L2_CID_WHITE_BALANCE_TEMPERATURE" << std::endl;
						return false;
				}
				return true;
		}

		[[nodiscard]] int get_white_balance_temperature() const override
		{
				struct v4l2_control ctrl = {0};
				ctrl.id									 = V4L2_CID_WHITE_BALANCE_TEMPERATURE;
				if(-1 == xioctl(_device_file_descriptor_, VIDIOC_G_CTRL, &ctrl))
				{
						std::cerr << "error getting V4L2_CID_WHITE_BALANCE_TEMPERATURE" << std::endl;
				}
				return ctrl.value;
		}

		[[nodiscard]] bool get_auto_white_balance_val() const override
		{
				struct v4l2_control ctrl = {0};
				ctrl.id									 = V4L2_CID_AUTO_WHITE_BALANCE;
				if(-1 == xioctl(_device_file_descriptor_, VIDIOC_G_CTRL, &ctrl))
				{
						std::cerr << "error getting V4L2_CID_AUTO_WHITE_BALANCE" << std::endl;
				}
				return ctrl.value;
		}

		bool set_auto_white_balance(bool enable) override
		{
				struct v4l2_control ctrl = {0};
				ctrl.id									 = V4L2_CID_AUTO_WHITE_BALANCE;
				ctrl.value							 = enable ? 1 : 0;
				if(-1 == xioctl(_device_file_descriptor_, VIDIOC_S_CTRL, &ctrl))
				{
						std::cerr << "error setting V4L2_CID_AUTO_WHITE_BALANCE" << std::endl;
						return false;
				}
				return true;
		}

		bool set_auto_exposure_mode(int type) override
		{
				/** V4L2_EXPOSURE_MANUAL and V4L2_EXPOSURE_APERTURE_PRIORITY are commonly
   * used. */
				struct v4l2_control ctrl = {0};
				ctrl.id									 = V4L2_CID_EXPOSURE_AUTO;
				ctrl.value							 = type;
				if(-1 == xioctl(_device_file_descriptor_, VIDIOC_S_CTRL, &ctrl))
				{
						std::cerr << "error setting V4L2_CID_EXPOSURE_AUTO" << std::endl;
						return false;
				}
				return true;
		}

		[[nodiscard]] int get_auto_exposure_current_value() const override
		{
				struct v4l2_control ctrl = {0};
				ctrl.id									 = V4L2_CID_EXPOSURE_AUTO;
				if(-1 == xioctl(_device_file_descriptor_, VIDIOC_G_CTRL, &ctrl))
				{
						return -1;
				}
				return ctrl.value;
		}

		bool enable_auto_exposure_auto_priority_mode(bool on) override
		{
				/**
     * The V4L2_CID_EXPOSURE_AUTO_PRIORITY is a control ID in the Video4Linux2
     * (V4L2) API, which is a part of the Linux kernel and provides an interface
     * to video devices. This control ID is a boolean type, meaning it can be
     * set to either true (1) or false (0). When the V4L2_CID_EXPOSURE_AUTO
     * control is set to AUTO or APERTURE_PRIORITY, the
     * V4L2_CID_EXPOSURE_AUTO_PRIORITY control determines if the device may
     * dynamically vary the frame rate. By default, this feature is disabled
     * (0), and the frame rate must remain constant. If
     * V4L2_CID_EXPOSURE_AUTO_PRIORITY is enabled (set to 1), the device can
     * adjust the frame rate dynamically, which can be useful in situations
     * where lighting conditions vary. In the context of still images, it is
     * recommended to set V4L2_CID_EXPOSURE_AUTO_PRIORITY to 1. This allows the
     * device to adjust the frame rate and exposure time dynamically to achieve
     * the best possible image quality.
     */

				struct v4l2_control ctrl = {0};
				ctrl.id									 = V4L2_CID_EXPOSURE_AUTO_PRIORITY;
				ctrl.value							 = on ? 1 : 0;
				if(-1 == xioctl(_device_file_descriptor_, VIDIOC_S_CTRL, &ctrl))
				{
						std::cerr << "error setting V4L2_CID_EXPOSURE_AUTO_PRIORITY" << std::endl;
						return false;
				}
				return true;
		}

		[[nodiscard]] bool is_auto_exposure_auto_priority_enabled() const override
		{
				struct v4l2_control ctrl = {0};
				ctrl.id									 = V4L2_CID_EXPOSURE_AUTO_PRIORITY;
				if(-1 == xioctl(_device_file_descriptor_, VIDIOC_G_CTRL, &ctrl))
				{
						std::cerr << "error getting V4L2_CID_EXPOSURE_AUTO_PRIORITY" << std::endl;
				}
				return ctrl.value;
		}

		bool set_manual_exposure_value(int val) override
		{
				/**
     * The exposure time is limited by the frame interval. Drivers should
     * interpret the values as 100 µs units, where the value 1 stands for
     * 1/10000th of a second, 10000 for 1 second and 100000 for 10 seconds. if
     * 30fps is the goal, each frame takes 1/30s = 33ms = 33*10*100us = 330 *
     * 100us. It depends on the ambient light to tuning exposure time.
     */

				struct v4l2_control ctrl = {0};
				set_auto_exposure_mode(V4L2_EXPOSURE_MANUAL);

				ctrl.id		 = V4L2_CID_EXPOSURE_ABSOLUTE;
				ctrl.value = val;
				if(-1 == xioctl(_device_file_descriptor_, VIDIOC_S_CTRL, &ctrl))
				{
						std::cerr << "error setting V4L2_CID_EXPOSURE_ABSOLUTE" << std::endl;
						return false;
				}
				return true;
		}

		[[nodiscard]] int get_manual_exposure_value() const override
		{
				struct v4l2_control ctrl = {0};
				ctrl.id									 = V4L2_CID_EXPOSURE_ABSOLUTE;
				if(-1 == xioctl(_device_file_descriptor_, VIDIOC_G_CTRL, &ctrl))
				{
						std::cerr << "error getting V4L2_CID_EXPOSURE_ABSOLUTE" << std::endl;
				}
				return ctrl.value;
		}

		[[nodiscard]] double set_fps(double fps) override
		{
				struct v4l2_streamparm stream;
				zero_that(stream);
				stream.type																	 = get_buffer_type_v4l2();
				stream.parm.capture.timeperframe.numerator	 = 1;
				stream.parm.capture.timeperframe.denominator = fps;
				if(-1 == xioctl(_device_file_descriptor_, VIDIOC_S_PARM, &stream))
				{
						std::cerr << "error setting set_fps" << std::endl;
						return 0;
				}

				if(stream.parm.capture.timeperframe.numerator)
				{
						const double fps_set = (double) stream.parm.capture.timeperframe.denominator
																	 / stream.parm.capture.timeperframe.numerator;
						if(are_floats_equal(fps, fps_set))
						{
								return 0;
						}
						else
						{
								return fps_set;
						}
				}
				return 0;
		}

		[[nodiscard]] double get_fps() const override
		{
				struct v4l2_streamparm stream_capabilities_query_payload;
				zero_that(stream_capabilities_query_payload);

				stream_capabilities_query_payload.type = get_buffer_type_v4l2();

				if(ioctl(_device_file_descriptor_, VIDIOC_G_PARM, &stream_capabilities_query_payload)
					 == 0)
				{
						if(stream_capabilities_query_payload.parm.capture.capability
							 & V4L2_CAP_TIMEPERFRAME)
						{
								return stream_capabilities_query_payload.parm.capture.timeperframe.denominator
											 / (double) stream_capabilities_query_payload.parm.capture.timeperframe
														 .numerator;
						}
				}
				return 0;
		}

		[[nodiscard]] unsigned int get_width() const override
		{
				return _v4l2_capture_format_.fmt.pix.width;
		}

		[[nodiscard]] unsigned int get_height() const override
		{
				return _v4l2_capture_format_.fmt.pix.height;
		}

	private:
		bool try_mmapped = true;
		std::vector<std::vector<std::pair<int, size_t>>> _buffer_dma_fds_;
		v4l2_format _v4l2_capture_format_;
		int _pixel_format_;
		int _device_file_descriptor_ = -1;
		v4l2_buf_type _buffer_plane_type_;
		unsigned int _num_buffers_ = 0;
		bool _limit_range_				 = false;
		std::vector<Multiplanar_Buffer> _allocated_buffers_;
		std::deque<v4l2_buffer> _buffer_of_buffers;
		std::string _device_dev_path_;
		std::vector<Multiplanar_Buffer_View> _mapped_buffers_;
};

} // namespace Cartrack

#endif // ISGURSOY_V4L2_HPP
