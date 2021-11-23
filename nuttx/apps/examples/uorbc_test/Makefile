############################################################################
# examples/uorbc_test/Makefile
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

include $(APPDIR)/Make.defs

# The uorbc_test example built-in application info
CSRCS += mod_uorbc_utils.c
CSRCS += topics_src/topic_test0.c topics_src/topic_test1.c topics_src/topic_test2.c topics_src/topic_test3.c topics_src/topic_test4.c 
CSRCS += topics_src/topic_test5.c topics_src/topic_test6.c topics_src/topic_test7.c topics_src/topic_test8.c topics_src/topic_test9.c topics_src/topic_test10.c 

PROGNAME = uorbc_test
PRIORITY = SCHED_PRIORITY_DEFAULT
STACKSIZE = $(CONFIG_DEFAULT_TASK_STACKSIZE)
MODULE = $(CONFIG_EXAMPLES_UORBC_TEST)

# The smallest thing you can build -- the uorbc_test example.

MAINSRC = uorbc_test_main.c



include $(APPDIR)/Application.mk
