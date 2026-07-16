#pragma once

#include <climits>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
}

namespace xgc_camera_driver {
namespace detail {

inline int checkedCompressedPayloadSize(const std::size_t size,
                                        const char* const format)
{
  if (size == 0U) {
    throw std::runtime_error(std::string("empty ") + format + " frame");
  }
  if (size > static_cast<std::size_t>(INT_MAX)) {
    throw std::runtime_error(std::string(format) +
                             " frame is too large for the decoder API");
  }
  return static_cast<int>(size);
}

// av_parser_parse2 may read up to AV_INPUT_BUFFER_PADDING_SIZE bytes beyond
// the logical packet. V4L2 MMAP buffers do not promise that padding, so never
// hand a dequeued frame directly to the parser.
class PaddedH264Input {
public:
  PaddedH264Input(const std::uint8_t* const data, const std::size_t size)
      : payload_size_(checkedCompressedPayloadSize(size, "H264")),
        bytes_(size + AV_INPUT_BUFFER_PADDING_SIZE, 0U)
  {
    if (!data) {
      throw std::runtime_error("H264 frame has a null payload");
    }
    std::memcpy(bytes_.data(), data, size);
  }

  const std::uint8_t* data() const noexcept { return bytes_.data(); }
  int payloadSize() const noexcept { return payload_size_; }
  const std::vector<std::uint8_t>& bytes() const noexcept { return bytes_; }

private:
  int payload_size_;
  std::vector<std::uint8_t> bytes_;
};

}  // namespace detail
}  // namespace xgc_camera_driver
