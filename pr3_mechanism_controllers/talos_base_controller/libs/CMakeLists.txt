add_library( AsyncSerial AsyncSerial.cpp )
add_library( BufferedAsyncSerial BufferedAsyncSerial.cpp )
target_link_libraries( BufferedAsyncSerial AsyncSerial )
