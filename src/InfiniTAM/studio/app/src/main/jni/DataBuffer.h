//
// Created by qkgautier on ...
//

#pragma once


#include <vector>
#include <mutex>


/**
 * Store data.
 * Thread-safe.
 */
template<class DataT>
class DataBuffer
{
public:

    DataBuffer(unsigned int buffer_size = 10):
            current_idx_(buffer_size-1)
    {
        buffer_.resize(buffer_size);
    }

    void addData(const DataT& data)
    {
        std::lock_guard<std::mutex> guard(pose_mutex_);

        unsigned int next_idx = (current_idx_ + 1) % buffer_.size();
        buffer_[next_idx] = data;
        current_idx_ = next_idx;
    }


    /**
     * Get one data.
     * Index 0 gives last data, index 1 gives data before last, etc.
     */
    DataT getData(unsigned int idx_from_last = 0) const
    {
        std::lock_guard<std::mutex> guard(pose_mutex_);

        idx_from_last = std::min(idx_from_last, (unsigned int)buffer_.size()-1);

        int idx = (current_idx_ - idx_from_last);
        if(idx < 0){ idx += buffer_.size(); }

        return buffer_[idx];
    }


    /**
     * Get multiple data, ranging from start_idx to end_idx.
     * Indices are backward, starting from latest data and going to older data, and are inclusive.
     * Results are stored by most recent data first, older data at the end.
     *
     * e.g. (start_idx, end_idx) = (0, 1) will give a vector [latest_data, second_latest_data]
     *
     * Reversing the indices will reverse the result order.
     */
    void getMultipleData(unsigned int start_from_last,
                         unsigned int end_from_last,
                         std::vector<DataT>& data) const
    {
        std::lock_guard<std::mutex> guard(pose_mutex_);

        start_from_last = std::min(start_from_last, (unsigned int)buffer_.size()-1);
        end_from_last   = std::min(end_from_last,   (unsigned int)buffer_.size()-1);

        int start_idx = (current_idx_ - start_from_last);
        int end_idx   = (current_idx_ - end_from_last);
        if(start_idx < 0){ start_idx += buffer_.size(); }
        if(end_idx < 0)  { end_idx   += buffer_.size(); }

        unsigned int size = abs((int)start_from_last - (int)end_from_last) + 1; // Cast to int important here!
        data.resize(size);

        int step = start_from_last <= end_from_last? -1: 1;
        for(int i = start_idx, j = 0; j < size; j++)
        {
            data[j] = buffer_[i];

            i += step;
            if(i < 0){ i += buffer_.size(); }
            i = i % buffer_.size();
        }
    }

    size_t getSize(){ return buffer_.size(); }

protected:

    std::vector<DataT> buffer_;
    unsigned int current_idx_;

    mutable std::mutex pose_mutex_;
};



