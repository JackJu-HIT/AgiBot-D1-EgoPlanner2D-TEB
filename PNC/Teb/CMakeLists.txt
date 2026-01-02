
include_directories(./include)
include_directories(./include/g2o_types)
include_directories(./include/suitesparse)
aux_source_directory(./src/ DIR_SRCS)

add_library (teb STATIC
            ${DIR_SRCS}
        )

