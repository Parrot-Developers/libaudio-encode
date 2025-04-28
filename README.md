# libaudio-encode - Audio encoding library

_libaudio-encode_ is a C library to handle the encoding of audio on
various platforms with a common API.

## Implementations

The following implementations are available:

* Fake AAC (generate silent AAC samples)
* Fraunhofer FDK AAC (software encoding)

The application can force using a specific implementation or let the library
decide according to what is supported by the platform.

## Dependencies

The library depends on the following Alchemy modules:

* libaac
* libaudio-defs
* libaudio-encode-core
* libfutils
* libmedia-buffers
* libmedia-buffers-memory
* libmedia-buffers-memory-generic
* libpomp
* libulog
* (optional) fdk-aac (for FDK AAC support)

## Building

Building is activated by enabling _libaudio-encode_ in the Alchemy build
configuration.

## Operation

Some encoders need the input buffers to originate from their own buffer
pool; when the input buffer pool returned by the library is not _NULL_ it must
be used and input buffers cannot be shared with other audio pipeline elements.

The encoding is asynchronous: the application pushes buffers to encode in the
input queue and is notified of encoded frames through a callback function.

### Threading model

The library is designed to run on a _libpomp_ event loop (_pomp_loop_, see
_libpomp_ documentation). All API functions must be called from the _pomp_loop_
thread. All callback functions (frame_output, flush or stop) are called from
the _pomp_loop_ thread.

## Testing

The library can be tested using the provided _aenc_ command-line tool which
takes as input a raw WAVE file and can optionally output an encoded AAC
bitstream.

To build the tool, enable _aenc_ in the Alchemy build configuration.

For a list of available options, run

    $ aenc -h
