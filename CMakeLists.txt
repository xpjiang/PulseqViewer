cmake_minimum_required(VERSION 3.20)

project(PulseqViewer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Open Qt Auto process
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(ENV{PATH} "${CMAKE_PREFIX_PATH}/bin;$ENV{PATH}")

set(PROJECT_ROOT ${CMAKE_SOURCE_DIR})
set(EXTERNAL_DIR ${CMAKE_SOURCE_DIR}/src/external)
set(PULSEQ_DIR ${CMAKE_SOURCE_DIR}/src/external/pulseq)
set(QCUSTOM_PLOT_DIR ${CMAKE_SOURCE_DIR}/src/external/qcustomplot)


SET(PULSEQ_LIST
${PULSEQ_DIR}/ExternalSequence.h
${PULSEQ_DIR}/ExternalSequence.cpp
)

SET(QCUSTOM_PLOT_LIST
${QCUSTOM_PLOT_DIR}/qcustomplot.h
${QCUSTOM_PLOT_DIR}/qcustomplot.cpp
)

SET(EXTERNAL_LIST
    ${PULSEQ_LIST}
    ${QCUSTOM_PLOT_LIST}
)
source_group("External" FILES ${EXTERNAL_LIST})

# find QT6
find_package(Qt6 COMPONENTS 
    Core
    Gui
    Widgets
    PrintSupport  # For QCustomPlot
    REQUIRED
)

file(GLOB UI_LIST "${PROJECT_ROOT}/src/*.ui")
source_group("Form" FILES ${UI_LIST})

file(GLOB SRC_LIST "${PROJECT_ROOT}/src/*.cpp")
file(GLOB HEADER_LIST "${PROJECT_ROOT}/src/*.h")
include_directories(${PULSEQ_DIR} ${QCUSTOM_PLOT_DIR})

add_executable(${PROJECT_NAME} ${SRC_LIST} ${HEADER_LIST} ${UI_LIST} ${EXTERNAL_LIST})

# Link Qt6
target_link_libraries(${PROJECT_NAME} PRIVATE 
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::PrintSupport)

set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME})
