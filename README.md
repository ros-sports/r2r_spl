# Robot-to-Robot communication in RoboCup SPL

[![Build and Test (humble)](../../actions/workflows/build_and_test_humble.yaml/badge.svg?branch=humble)](../../actions/workflows/build_and_test_humble.yaml?query=branch:humble)
[![Build and Test (iron)](../../actions/workflows/build_and_test_iron.yaml/badge.svg?branch=iron)](../../actions/workflows/build_and_test_iron.yaml?query=branch:iron)
[![Build and Test (rolling)](../../actions/workflows/build_and_test_rolling.yaml/badge.svg?branch=rolling)](../../actions/workflows/build_and_test_rolling.yaml?query=branch:rolling)

ROS2 package that handles intra-team communication using SPLStandardMessage defined in the RoboCup SPL rulebook.

For more information, see our [Documentation](https://robot2robot-spl.readthedocs.io/)


# Branches

Note that this repository has an **unusual branching strategy**.
The default branch is the **dev** branch.

The **dev** branch is unreleased, and should work with SPL GameController's master branch's SPLStandardMessage example.
When a new SPLStandardMessage release is made (eg. version 8), a new package will be created (eg.r2r_spl_8) and be backported to all active distros (eg. rolling, humble, etc.) and released.

That way, teams can use new versions of SPLStandardMessage on distros apart from Rolling!

None of the distro branches should contain the r2r_spl_master package.
When a distro is being branched off, make sure to delete the r2r_spl_master package on that branch because it will always have API breaking changes.
The reason there is a separate dev branch, is because if rolling is the dev branch, then when a new distro is released (branched off from rolling), it will contain r2r_spl_master. (which is an issue)
