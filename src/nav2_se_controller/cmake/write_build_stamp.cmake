# Copyright (c) 2026 Jungmo Kang
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Writes the build stamp: the git state of the tree that is being built.
#   cmake -DSRC=<source dir> -DOUT=<stamp file> -P write_build_stamp.cmake
# Campaign 2 refuses a counted trial unless the installed stamp says
# git_describe=campaign2-controller (the tag itself, not dirty) and git_head=<tag commit>.
execute_process(COMMAND git -C "${SRC}" describe --tags --always --dirty
  OUTPUT_VARIABLE SE_DESCRIBE OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET RESULT_VARIABLE SE_RC1)
execute_process(COMMAND git -C "${SRC}" rev-parse HEAD
  OUTPUT_VARIABLE SE_HEAD OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET RESULT_VARIABLE SE_RC2)
if(NOT SE_RC1 EQUAL 0)
  set(SE_DESCRIBE "unknown")
endif()
if(NOT SE_RC2 EQUAL 0)
  set(SE_HEAD "unknown")
endif()
file(WRITE "${OUT}" "git_describe=${SE_DESCRIBE}\ngit_head=${SE_HEAD}\n")
