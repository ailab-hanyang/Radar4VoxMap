#!/bin/bash

# remove build folder
(sudo rm -rf build/)
(sudo rm -rf lib/)
(sudo rm -rf bin/)

# remove local include folder
(sudo rm -rf /usr/local/include/g2o)
# remove local lib files
(sudo rm -rf /usr/local/lib/libg2o*)
(sudo rm -rf /usr/local/lib/cmake/g2o)