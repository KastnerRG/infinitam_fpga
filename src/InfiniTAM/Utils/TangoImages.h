//
// Created by qkgautier on 5/13/16.
//

#ifndef RECORDSENSORDATA_TANGOIMAGES_H
#define RECORDSENSORDATA_TANGOIMAGES_H


#include <stdint.h>
#include <vector>

#include "tango_client_api.h"


// TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
// TODO This is not thread-safe!

class TangoImages
{
public:
    /**
     * Construct a buffer of size 0 (uninitialized).
     * @warning You need to initialize the buffer when using this constructor.
     * Calling functions on uninitialized buffer results in undefined behavior.
     */
    TangoImages();


    ~TangoImages();

    /**
     * Construct and initialize the buffer.
     */
    TangoImages(unsigned int buffer_size,
                unsigned int max_width = 1280,
                unsigned int max_height = 720);

    /**
     * Initialize the buffer to the requested size.
     */
    void initialize(unsigned int buffer_size = 10,
                    unsigned int max_width = 1280,
                    unsigned int max_height = 720);

    unsigned int size() const{ return frame_buffer_.size(); }

    void addNewData(const TangoImageBuffer* data);

    TangoImageBuffer* getCurrentData();

    TangoImageBuffer* getDataByTimestamp(double target_timestamp);


protected:

    std::vector< TangoImageBuffer > frame_buffer_;

    unsigned int current_idx_;
};


#endif //RECORDSENSORDATA_TANGOIMAGES_H
