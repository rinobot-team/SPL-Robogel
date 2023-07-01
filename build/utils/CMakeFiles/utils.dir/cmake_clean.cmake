file(REMOVE_RECURSE
  "libutils.a"
  "libutils.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/utils.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
