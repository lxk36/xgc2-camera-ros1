#include "h264_input_buffer.hpp"

#include <climits>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

#include <gtest/gtest.h>

namespace detail = xgc_camera_driver::detail;

TEST(H264InputBuffer, CopiesPayloadAndProvidesZeroParserPadding)
{
  const std::uint8_t payload[] = {0x00, 0x00, 0x01, 0x67, 0xaa, 0x55};
  const detail::PaddedH264Input input(payload, sizeof(payload));

  ASSERT_EQ(input.payloadSize(), static_cast<int>(sizeof(payload)));
  ASSERT_EQ(input.bytes().size(), sizeof(payload) + AV_INPUT_BUFFER_PADDING_SIZE);
  for (std::size_t index = 0; index < sizeof(payload); ++index) {
    EXPECT_EQ(input.bytes()[index], payload[index]);
  }
  for (std::size_t index = sizeof(payload); index < input.bytes().size(); ++index) {
    EXPECT_EQ(input.bytes()[index], 0U);
  }
}

TEST(H264InputBuffer, RejectsInvalidDecoderSizes)
{
  EXPECT_THROW(detail::checkedCompressedPayloadSize(0U, "H264"), std::runtime_error);
  EXPECT_THROW(detail::checkedCompressedPayloadSize(
                   static_cast<std::size_t>(INT_MAX) + 1U, "H264"),
               std::runtime_error);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
