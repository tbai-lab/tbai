add_subdirectory(tbai_static)

if(TBAI_BUILD_BOB)
  add_subdirectory(tbai_bob)
endif()

if(TBAI_BUILD_WTW)
  add_subdirectory(tbai_wtw)
endif()

if(TBAI_BUILD_NP3O)
  add_subdirectory(tbai_np3o)
endif()

if(TBAI_BUILD_MPC)
  add_subdirectory(tbai_mpc)
endif()

if(TBAI_BUILD_DTC)
  add_subdirectory(tbai_dtc)
endif()

if(TBAI_BUILD_JOE)
  add_subdirectory(tbai_joe)
endif()
