#!/bin/bash

echo "Setting up KVS"

KVS_SDK_ROOT="$KVS_SDK_ROOT_FOLDER"

mkdir -p $KVS_SDK_ROOT
cd $KVS_SDK_ROOT

git clone https://github.com/awslabs/amazon-kinesis-video-streams-producer-sdk-cpp.git
git clone --recursive https://github.com/awslabs/amazon-kinesis-video-streams-webrtc-sdk-c.git

mkdir -p amazon-kinesis-video-streams-producer-sdk-cpp/build
cd amazon-kinesis-video-streams-producer-sdk-cpp/build

cmake .. -DBUILD_GSTREAMER_PLUGIN=TRUE
make

cd ../

export GST_PLUGIN_PATH=`pwd`/build
export LD_LIBRARY_PATH=`pwd`/open-source/local/lib

# check if GST_PLUGIN_PATH and LD_LIBRARY_PATH are set correctly in ~/.bashrc else set them

echo "GST_PLUGIN_PATH: $GST_PLUGIN_PATH"
echo "LD_LIBRARY_PATH: $LD_LIBRARY_PATH"

# if ! grep -q "export GST_PLUGIN_PATH" ~/.bashrc; then
#     echo "export GST_PLUGIN_PATH=`pwd`/build" >> ~/.bashrc
#     echo "LD_LIBRARY_PATH set in ~/.bashrc"
# fi

# if ! grep -q "export LD_LIBRARY_PATH" ~/.bashrc; then
#     echo "export LD_LIBRARY_PATH=`pwd`/open-source/local/lib" >> ~/.bashrc
#     echo "LD_LIBRARY_PATH set in ~/.bashrc"
# fi
# source ~/.bashrc

echo "export GST_PLUGIN_PATH=`pwd`/build" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=`pwd`/open-source/local/lib" >> ~/.bashrc

echo "KVS setup complete"

cd $KVS_SDK_ROOT

mkdir -p amazon-kinesis-video-streams-webrtc-sdk-c/build
cd amazon-kinesis-video-streams-webrtc-sdk-c/build

cmake ..
make