/**
 * @file network_t.hpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Colin Rennie
 *
 * Email: pracsys@googlegroups.com
 *
 * Originally adapted from author Tzutalin: 
 * https://github.com/tzutalin/ros_caffe
 */
#pragma once

#ifndef NETWORK_T_H
#define NETWORK_T_H

#include <iostream>
#include <vector>
#include <sstream>
#include <caffe/caffe.hpp>

using namespace caffe;
using std::string;

template <class T> 
class network_t {
    public:
        network_t(const string& model_file,
                   const string& trained_file,
                   const string& mean_file,
                   const string& min_file,
                   const string& max_file);

        T forward_pass(T input);

    private:
        void SetMean(const string& mean_file);

        void SetMin(const string& mean_file);
        
        void SetMax(const string& mean_file);

        void normalize(T& data);

        T forward(T data);

    private:
        shared_ptr<Net<float> > net_;
        int inputs_;
        int channels_;
        int width_;
        int height_;
        T mean_;
        T min_;
        T max_;
};

template <class T>
network_t<T>::network_t(const string& model_file,
                       const string& trained_file,
                       const string& mean_file,
                       const string& min_file,
                       const string& max_file) {
#ifdef CPU_ONLY
    Caffe::set_mode(Caffe::CPU);
#else
    Caffe::set_mode(Caffe::GPU);
#endif

    // Load the network 
    net_.reset(new Net<float>(model_file, TEST));
    
    // Load Weights 
    net_->CopyTrainedLayersFrom(trained_file);
    
    // Get input blob pointer and dimensions of input
    Blob<float>* input_layer = net_->input_blobs()[0];

    // Load dimensions of network
    inputs_ = input_layer->num();
    channels_ = input_layer->channels();
    width_ = input_layer->width();
    height_ = input_layer->height();
    
    // Load protobuf blob files for data normalization 
    SetMean(mean_file);
    SetMin(min_file);
    SetMax(max_file);

    // Get output blob pointer
    Blob<float>* output_layer = net_->output_blobs()[0];
}

/* Return the top N predictions. */
template <class T>
T network_t<T>::forward_pass(T input) {
    T output = forward(input);
    return output;
}

/* Load the mean file in binaryproto format. */
template <class T>
void network_t<T>::SetMean(const string& mean_file) {
    BlobProto blob_proto;
    ReadProtoFromBinaryFileOrDie(mean_file.c_str(), &blob_proto);
    
    Blob<float> mean_blob;
    mean_blob.FromProto(blob_proto);
    
    float* data = mean_blob.mutable_cpu_data();
    for (int i = 0; i < mean_blob.channels(); i++) {
        mean_.push_back(*data);
        data += 1;
    }
}

template <class T>
void network_t<T>::SetMin(const string& min_file) {
    BlobProto blob_proto;
    ReadProtoFromBinaryFileOrDie(min_file.c_str(), &blob_proto);

    /* Convert from BlobProto to Blob<float> */
    Blob<float> min_blob;
    min_blob.FromProto(blob_proto);

    float* data = min_blob.mutable_cpu_data();
    for (int i = 0; i < min_blob.channels(); i++) {
        min_.push_back(*data);
        data += 1;
    }
}

template <class T>
void network_t<T>::SetMax(const string& max_file) {
    BlobProto blob_proto;
    ReadProtoFromBinaryFileOrDie(max_file.c_str(), &blob_proto);

    /* Convert from BlobProto to Blob<float> */
    Blob<float> max_blob;
    max_blob.FromProto(blob_proto);

    float* data = max_blob.mutable_cpu_data();
    for (int i = 0; i < max_blob.channels(); i++) {
        max_.push_back(*data);
        data += 1;
    }
}

/*
This will be the FORWARD PASS operation on input data
*/
template <class T>
T network_t<T>::forward(T input) {
    Blob<float>* input_layer = net_->input_blobs()[0];
    input_layer->Reshape(1, channels_, height_, width_);
    
    /* Forward dimension change to all layers. */
    net_->Reshape();

    normalize(input);

    float* input_data = input_layer->mutable_cpu_data();
    for (int i = 0; i < input_layer->channels(); i++) {
        *input_data = input[i];
        input_data += (input_layer->width() * input_layer->height());
    }

    net_->ForwardPrefilled();

    /* Copy the output layer to the template structure */
    Blob<float>* output_layer = net_->output_blobs()[0];

    const float* begin = output_layer->cpu_data();
    const float* end = begin + output_layer->channels();
    T retval(begin, end);

    return retval;
}

/*
This will be where we normalize the data prior to the forward pass
*/
template <class T>
void network_t<T>::normalize(T& data) {
    // Normalize the data to these desired bounds
    int desired_min = -1, desired_max = 1;
    
    for (int i = 0; i < data.size()-1; i++) {
        data[i] = (((data[i] - mean_[i] - min_[i]) 
            * (desired_max - desired_min))/(max_[i] - min_[i])) + desired_min;
    }
}



#endif
