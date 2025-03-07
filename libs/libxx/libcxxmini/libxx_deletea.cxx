//***************************************************************************
// libs/libxx/libcxxmini/libxx_deletea.cxx
//
// Licensed to the Apache Software Foundation (ASF) under one or more
// contributor license agreements.  See the NOTICE file distributed with
// this work for additional information regarding copyright ownership.  The
// ASF licenses this file to you under the Apache License, Version 2.0 (the
// "License"); you may not use this file except in compliance with the
// License.  You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
// WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
// License for the specific language governing permissions and limitations
//
//***************************************************************************

//***************************************************************************
// Included Files
//***************************************************************************

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <cstddef>

#include <nuttx/lib/lib.h>

//***************************************************************************
// Operators
//***************************************************************************

//***************************************************************************
// Name: delete[]
//***************************************************************************

void operator delete[](FAR void *ptr) throw()
{
  lib_free(ptr);
}

#ifdef CONFIG_HAVE_CXX14

//***************************************************************************
// Operators
//***************************************************************************

//***************************************************************************
// Name: delete[]
//***************************************************************************

void operator delete[](FAR void *ptr, std::size_t size)
{
  lib_free(ptr);
}

#endif /* CONFIG_HAVE_CXX14 */
