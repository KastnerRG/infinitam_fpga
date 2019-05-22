//
// Created by qkgautier on 5/13/16.
//

#include "TangoImages.h"
#include <algorithm>


using namespace std;

TangoImages::TangoImages():
        current_idx_(0)
{ }

TangoImages::~TangoImages()
{
    for(auto& b: frame_buffer_)
    {
        delete[] b.data;
    }
    frame_buffer_.clear();
}

TangoImages::TangoImages(unsigned int buffer_size,
                         unsigned int max_width,
                         unsigned int max_height)
{
    initialize(buffer_size, max_width, max_height);
}

void TangoImages::initialize(unsigned int buffer_size,
                             unsigned int max_width,
                             unsigned int max_height)
{
    current_idx_ = 0;

    if(buffer_size == 0){ buffer_size = 1; }

    frame_buffer_.clear();
    frame_buffer_.resize(buffer_size);

    for(auto& b: frame_buffer_)
    {
        b.data = new uint8_t[int(max_width*max_height*1.5)];
        b.height = 1;
        b.width  = 1;
    }
}

void TangoImages::addNewData(const TangoImageBuffer * data)
{
    unsigned int next_idx = (current_idx_ + 1) % frame_buffer_.size();

    size_t buf_size = int(data->width * data->height * 1.5);

    frame_buffer_[next_idx].format       = data->format;
    frame_buffer_[next_idx].frame_number = data->frame_number;
    frame_buffer_[next_idx].height       = data->height;
    frame_buffer_[next_idx].stride       = data->stride;
    frame_buffer_[next_idx].timestamp    = data->timestamp;
    frame_buffer_[next_idx].width        = data->width;

    std::copy(data->data, data->data + buf_size, frame_buffer_[next_idx].data);

    current_idx_ = next_idx;
}


TangoImageBuffer * TangoImages::getCurrentData()
{
    return &frame_buffer_[current_idx_];
}

TangoImageBuffer * TangoImages::getDataByTimestamp(double target_timestamp)
{
    unsigned int start_idx = (current_idx_ + 1) % frame_buffer_.size();

    // Could do a binary search here, but for 10 elements it should be fine



    unsigned int i;
    for(i = start_idx; i != current_idx_; i=(i+1)%frame_buffer_.size())
    {
        if(target_timestamp <= frame_buffer_[i].timestamp){ break; }
    }

    unsigned int prev_i = i - 1;
    if(i == 0){ prev_i = frame_buffer_.size()-1; }

    if(abs(target_timestamp-frame_buffer_[prev_i].timestamp) < abs(target_timestamp-frame_buffer_[i].timestamp))
    { i = prev_i; }


    return &frame_buffer_[i];
}









