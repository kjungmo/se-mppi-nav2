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
