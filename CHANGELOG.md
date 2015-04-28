Change log
==========

1.1.0 (2015-04-xx)
==================

* API changes:
  * Change to SI units, i.e. translations are now assumed to be in meters, not millimeters.
  * Encoder/pots discrepencies are now treated as errors.
  * Requires mechatronics library Amp1394 version 1.1 or higher.
  * Removed encoder bits to position offset, now assumes the offset if preloaded on FPGA
* Deprecated features:
* New features:
  * Added `Error`, `Warning` and `Status` events with `std::string` payload.
  * Preload encoders when component starts.
  * All encoders are now relative to mid-range to avoid overflow.
  * Added detection of encoder bit overflow.
  * C++ exceptions are now caught and trigger cisstMultiTask events `Error`s.
* Bug fixes:


1.0.1 (2014-12-30)
==================

* No change log file, initial release.
