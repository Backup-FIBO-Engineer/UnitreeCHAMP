#!/usr/bin/env bash
# Offline (no ROS runtime) checks for every robot described in this package,
# or only for the robots given on the command line:
#
#   bash tools/run_offline_checks.sh            # all robots in urdf/
#   bash tools/run_offline_checks.sh go2 b2     # selected robots
#
# A robot is urdf/<robot>.urdf|.xacro + config/<robot>_{gait,joints,links}.yaml.
# MuJoCo checks run when mujoco/<robot>.xml exists, the LowCmd check when
# config/<robot>_lowcmd.yaml exists, the IMU body pose loop check when
# config/<robot>_body_pose.yaml exists. Needs python3 (numpy, PyYAML, mujoco),
# g++, pkg-config, tinyxml2 and yaml-cpp dev packages. verify_unitree_lowcmd
# also needs urdfdom (found through pkg-config or a sourced ROS 2 prefix) and
# is skipped with a warning otherwise.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TOOLS="${ROOT}/tools"
CHAMP_INCLUDE="${CHAMP_INCLUDE:-${ROOT}/../champ/include/champ}"
BUILD_DIR="${CHAMP_OFFLINE_BUILD_DIR:-/tmp/champ_offline_checks}"
mkdir -p "${BUILD_DIR}"
cd "${ROOT}"

if [ $# -gt 0 ]; then
  ROBOTS=("$@")
else
  mapfile -t ROBOTS < <(python3 "${TOOLS}/champ_robot_files.py" list "${ROOT}")
fi
if [ ${#ROBOTS[@]} -eq 0 ]; then
  echo "no robots found in ${ROOT}/urdf" >&2
  exit 1
fi

step() { echo; echo "=== $* ==="; }

# --- compile the C++ tools once ---------------------------------------------
step "compile C++ tools"
CXXFLAGS_COMMON=(-std=c++17 -O2 -I "${CHAMP_INCLUDE}" -I "${TOOLS}")
XML_YAML_CFLAGS=($(pkg-config --cflags tinyxml2 yaml-cpp))
XML_YAML_LIBS=($(pkg-config --libs tinyxml2 yaml-cpp))
g++ "${CXXFLAGS_COMMON[@]}" "${XML_YAML_CFLAGS[@]}" \
  -o "${BUILD_DIR}/verify_champ_robot" tools/verify_champ_robot.cpp "${XML_YAML_LIBS[@]}"
g++ "${CXXFLAGS_COMMON[@]}" "${XML_YAML_CFLAGS[@]}" \
  -o "${BUILD_DIR}/dump_champ_gait" tools/dump_champ_gait.cpp "${XML_YAML_LIBS[@]}"
g++ "${CXXFLAGS_COMMON[@]}" -I include "${XML_YAML_CFLAGS[@]}" \
  -o "${BUILD_DIR}/verify_body_pose_controller" tools/verify_body_pose_controller.cpp \
  "${XML_YAML_LIBS[@]}"

LOWCMD_BIN=""
URDFDOM_CFLAGS=()
URDFDOM_LIBS=()
if pkg-config --exists urdfdom 2>/dev/null; then
  URDFDOM_CFLAGS=($(pkg-config --cflags urdfdom))
  URDFDOM_LIBS=($(pkg-config --libs urdfdom))
elif [ -n "${AMENT_PREFIX_PATH:-}" ]; then
  for prefix in ${AMENT_PREFIX_PATH//:/ }; do
    if [ -d "${prefix}/include/urdfdom" ]; then
      URDFDOM_CFLAGS=(-I "${prefix}/include/urdfdom" -I "${prefix}/include/urdfdom_headers")
      libdir="$(dirname "$(find "${prefix}/lib" -name 'liburdfdom_model.so*' | head -n1)")"
      URDFDOM_LIBS=(-L "${libdir}" -Wl,-rpath,"${libdir}" -lurdfdom_model)
      break
    fi
  done
fi
if [ ${#URDFDOM_LIBS[@]} -gt 0 ]; then
  g++ -std=c++17 -O2 -I include "${URDFDOM_CFLAGS[@]}" $(pkg-config --cflags yaml-cpp) \
    -o "${BUILD_DIR}/verify_unitree_lowcmd" tools/verify_unitree_lowcmd.cpp src/motor_crc.cpp \
    "${URDFDOM_LIBS[@]}" $(pkg-config --libs yaml-cpp)
  LOWCMD_BIN="${BUILD_DIR}/verify_unitree_lowcmd"
else
  echo "[WARN] urdfdom not found (pkg-config or sourced ROS prefix); LowCmd checks are skipped"
fi

# --- per-robot checks --------------------------------------------------------
for robot in "${ROBOTS[@]}"; do
  gait="config/${robot}_gait.yaml"
  joints="config/${robot}_joints.yaml"
  links="config/${robot}_links.yaml"
  if [ -f "urdf/${robot}.urdf" ]; then
    urdf="urdf/${robot}.urdf"
  elif [ -f "urdf/${robot}.xacro" ]; then
    urdf="${BUILD_DIR}/${robot}.urdf"
    python3 "${TOOLS}/champ_robot_files.py" expand "urdf/${robot}.xacro" "${urdf}"
  else
    echo "unknown robot '${robot}' (no urdf/${robot}.urdf or .xacro)" >&2
    exit 1
  fi

  step "${robot}: URDF -> CHAMP contract"
  python3 tools/verify_urdf_champ_contract.py --robot "${robot}"

  step "${robot}: CHAMP IK/FK/gait/odometry"
  "${BUILD_DIR}/verify_champ_robot" "${urdf}" "${gait}" "${joints}" "${links}"

  if [ -f "mujoco/${robot}.xml" ]; then
    step "${robot}: MuJoCo model vs URDF"
    python3 tools/validate_mujoco_against_urdf.py --robot "${robot}"
    step "${robot}: MuJoCo standing physics"
    python3 tools/verify_mujoco_physics.py --robot "${robot}"
    step "${robot}: MuJoCo forward walk"
    python3 tools/verify_mujoco_walk.py --robot "${robot}"
  else
    echo "[INFO] ${robot}: no mujoco/${robot}.xml, MuJoCo checks skipped"
  fi

  if [ -f "config/${robot}_lowcmd.yaml" ]; then
    if [ -n "${LOWCMD_BIN}" ]; then
      step "${robot}: Unitree LowCmd mapping"
      "${LOWCMD_BIN}" "${urdf}" "${joints}"
    else
      echo "[WARN] ${robot}: verify_unitree_lowcmd skipped (urdfdom missing)"
    fi
  fi

  if [ -f "config/${robot}_body_pose.yaml" ]; then
    step "${robot}: IMU body roll/pitch loop"
    "${BUILD_DIR}/verify_body_pose_controller" "${urdf}" "${gait}" "${joints}" "${links}" \
      "config/${robot}_body_pose.yaml"
  fi
done

echo
echo "All offline checks passed for: ${ROBOTS[*]}"
