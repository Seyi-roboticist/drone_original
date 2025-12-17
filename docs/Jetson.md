# ROS 2 Humble Installation on Jetson Nano: Challenges and Decision Summary

## Hardware Specifications

We initially attempted to run ROS 2 Humble on the following hardware:

- **Device**: NVIDIA Jetson Nano 4GB Developer Kit  
- **Production Module**: P3448-0002  
- **Carrier Board**: P3449-0000-B00  
- **Processor**: Quad-core ARM Cortex-A57 (ARMv8 rev 1 × 4)  
- **GPU**: NVIDIA Tegra X1 (T210)  
- **Operating System**: JetPack 4.6.4 (Ubuntu 18.04, Linux Kernel 4.9)

JetPack 4.6.4 is the latest officially supported version for the Jetson Nano by NVIDIA.

---

## Attempt 1: Running ROS 2 Humble on Ubuntu 18.04

ROS 2 Humble is not officially supported on Ubuntu 18.04. Our attempt to install and run it faced multiple challenges:

- No official apt package support; all dependencies must be manually downloaded and built from source.
- Compatibility issues emerged between ROS 2 Humble packages and system libraries in Ubuntu 18.04.
- Running ROS 2 packages resulted in instability and unpredictable behavior.
- Docker-based installation was also attempted but failed to resolve core compatibility problems.

---

## Attempt 2: Upgrading to Ubuntu 22.04

Since ROS 2 Humble is officially supported on Ubuntu 22.04, we explored this option using a community-supported custom image. The image was successfully installed, but introduced several critical issues:

- **GPU Driver Incompatibility**:  
  The NVIDIA GPU driver for Tegra X1 is incompatible with newer versions of the X.Org server used in Ubuntu 22.04. We were unable to find a stable combination that worked.
  
- **CUDA Limitations**:  
  Jetson Nano officially supports up to CUDA 10.2. This older version is incompatible with many GPU-accelerated applications on Ubuntu 22.04.
  
- **Performance Degradation**:  
  Ubuntu 22.04 demands significantly more system resources, leading to poor performance on the Jetson Nano.
  
- **System Instability**:  
  Basic functionalities such as display brightness control were unreliable, and numerous error logs appeared during system startup.

These limitations rendered the system too slow and unstable for practical use with ROS 2.

---

## Final Decision

Due to the significant hardware and software limitations encountered with both Ubuntu 18.04 and 22.04:

- We reverted the Jetson Nano to its original JetPack 4.6.4 (Ubuntu 18.04) configuration for potential future non-ROS2 tasks.
- We opted to use a separate onboard computer with full ROS 2 Humble and GPU support for ongoing development.

---

## Conclusion

While the Jetson Nano remains a capable platform for certain applications, it is currently not suitable for running ROS 2 Humble in a stable and efficient manner. Developers targeting ROS 2 on resource-constrained devices are advised to use hardware platforms with official Ubuntu 22.04 and CUDA support.
