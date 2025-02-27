Questions:
* Which package
* Which node
* Service names?

Failures:
* Any time we write files, this could fail.
* This writes files at three times:
  * Create new file
  * Add new position
  * Delete position
* The program may be interrupted (shut down) or the power may be interrupted at any time. As a result, any of these can be interrupted, resulting in:
  * Create new file: Invalid header format
  * Add new position: Invalid entry
  * Delete position:
    * Current implementation: Rewrites file, so in-progress entry is invalid and all entries not written are lost.
    * In-place implementation: ???
* In any case, the operating system handles file errors (so files will always be valid, but their contents may not be).

# Plan
startup:
* declare filename as parameter
* if file exists read it into list

on run, service accepts:
* sensor_msgs/NavSatFix

on run, service:
* writes line to array
* writes line to file

read lines from file:
* if file exists:
* for every line in the file:
  * log error for malformed line and don't load it, then keep running
  * append it to the list

file should be .csv

save_current_position
* check status/NO_FIX -> if true, log error (and timestamp/location)
  * should I still save this to the file? I think maybe?
* write line to list/file:
  * latitude, longitude, timestamp, (error maybe)
