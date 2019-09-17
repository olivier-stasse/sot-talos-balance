SET(${LIBRARY_NAME}_HEADERS
  utils/commands-helper.hh
  utils/statistics.hh
  math/fwd.hh
  robot/fwd.hh
  robot/robot-wrapper.hh
  sdk_qualisys/Network.h
  sdk_qualisys/RTPacket.h
  sdk_qualisys/RTProtocol.h
  sdk_qualisys/Markup.h
  )

INSTALL(FILES ${${LIBRARY_NAME}_HEADERS} DESTINATION include/${CUSTOM_HEADER_DIR})
