settings {
  time_display = relative
  completion_cache_ttl = 10min
  registry_timeout = 4s
}

compression {
  pointcloud_mode = lossy
  pointcloud_accuracy = 1mm
  packed_mcap_compression = zstd
  packed_archive_compression = none
  unpacked_mcap_compression = lz4
}
