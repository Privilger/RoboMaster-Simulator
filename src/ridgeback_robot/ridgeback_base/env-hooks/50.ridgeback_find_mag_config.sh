RIDGEBACK_MAG_CONFIG=$(catkin_find --etc --first-only ridgeback_base mag_config.yaml 2>/dev/null)
if [ -z "$RIDGEBACK_MAG_CONFIG" ]; then
  RIDGEBACK_MAG_CONFIG=$(catkin_find --share --first-only ridgeback_base config/mag_config_default.yaml 2>/dev/null)
fi

export RIDGEBACK_MAG_CONFIG
