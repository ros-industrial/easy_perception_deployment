This document is a declaration of software quality for the `easy_perception_deployment` package, based on the guidelines in [REP-2004](https://github.com/ros-infrastructure/rep/blob/master/rep-2004.rst).

# `easy_perception_deployment` Quality Declaration

The package `easy_perception_deployment` claims to be in the **Quality Level 5** category.

Below are the rationales, notes, and caveats for this claim, organized by each requirement listed in the [Package Quality Categories in REP-2004](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#package-quality-categories) of the ROS2 developer guide.

## Version Policy [1]

### Version Scheme [1.i]
`easy_perception_deployment` uses `semver` according to the recommendation for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#versioning)

### Version Stability [1.ii]

`easy_perception_deployment` is at a stable version, i.e. `>= 1.0.0`.
The current version can be found in its [package.xml](package.xml), and its change history can be found in its [CHANGELOG](CHANGELOG.rst).

### Public API Declaration [1.iii]
All symbols in the installed headers are considered part of the public API.

All installed headers are in the `include` directory of the package, headers in any other folders are not installed and considered private.

Additionally, there are generated header files which are installed and therefore part of the public API.
The source templates for these generated headers are in the `resource` folder.

### API Stability Policy [1.iv]

`easy_perception_deployment` will not break public API within a released ROS distribution, i.e. no major releases once the ROS distribution is released.

### ABI Stability Policy [1.v]

`easy_perception_deployment` contains C code and therefore must be concerned with ABI stability, and will maintain ABI stability within a ROS distribution.

### ABI and ABI Stability Within a Released ROS Distribution [1.vi]

`easy_perception_deployment` will not break API nor ABI within a released ROS distribution, i.e. no major releases once the ROS distribution is released.

## Change Control Process [2]

`easy_perception_deployment` follows the recommended guidelines for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#change-control-process).

### Change Requests [2.i]
All changes will occur through a pull request, check [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#change-control-process) for additional information.

### Contributor Origin [2.ii]
This package uses DCO as its confirmation of contributor origin policy. More information can be found in [CONTRIBUTING](./CONTRIBUTING.md)

### Peer Review Policy [2.iii]
All pull request will be peer-reviewed, check [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#change-control-process) for additional information.

### Continuous Integration [2.iv]

All pull request must pass CI. To Elaborate.

###  Documentation Policy [2.v]

All pull requests must resolve related documentation changes before merging

## Documentation [3]

### Feature Documentation [3.i]

`easy_perception_deployment` has a documented feature list and it is hosted [here]().

### Public API Documentation [3.ii]

`easy_perception_deployment` has documentation of its public API and it is hosted [here]().

### License [3.iii]

The license for `easy_perception_deployment` is Apache 2.0, and a summary is in each source file, the type is declared in the [`package.xml`]() manifest file, and a full copy of the license is in the [`LICENSE`]() file.

There is an automated test which runs ament_copyright that ensures each file has a license statement.

### Copyright Statements [3.iv]

The copyright holders each provide a statement of copyright in each source code file in `easy_perception_deployment`.

There is an automated test which runs a linter that ensures each file has at least one copyright statement. Latest linter result report can be seen [here]().

## Testing [4]

### Feature Testing [4.i]

Each feature in `easy_perception_deployment` has corresponding tests which simulate typical usage, and they are located in the [`test`]() directory.

### Public API Testing [4.ii]

Each part of the public API has tests, and new additions or changes to the public API require tests before being added.
The tests aim to cover both typical usage and corner cases, but are quantified by contributing to code coverage.

### Coverage [4.iii]

`easy_perception_deployment` follows the recommendations for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#code-coverage), and opts to use line coverage instead of branch coverage.

This includes:

- tracking and reporting line coverage statistics
- achieving and maintaining a reasonable branch line coverage (90-100%)
- no lines are manually skipped in coverage calculations

Changes are required to make a best effort to keep or increase coverage before being accepted, but decreases are allowed if properly justified and accepted by reviewers.

### Performance [4.iv]

To be elaborated.

### Linters and Static Analysis [4.v]

To be elaborated.

## Dependencies [5]

To be elaborated.

### Direct Runtime ROS Dependencies [5.i]
`easy_perception_deployment` has no run-time or build-time dependencies that need to be considered for this declaration.

It has several "buildtool" dependencies, which do not affect the resulting quality of the package, because they do not contribute to the public library API.

### Optional Direct Runtime ROS Dependencies [5.ii]
`easy_perception_deployment` has no run-time or build-time dependencies that need to be considered for this declaration.

### Direct Runtime non-ROS Dependency [5.iii]
`easy_perception_deployment` has no run-time or build-time dependencies that need to be considered for this declaration.

## Platform Support [6]

To be elaborated.

## Vulnerability Disclosure Policy [7.i]

To be elaborated.

# Current status Summary

The chart below compares the requirements in the REP-2004 with the current state of the easy_perception_deployment package.

|Number|  Requirement| Current state |
|--|--|--|
|1| **Version policy** |---|
|1.i|Version Policy available | ✓ |
|1.ii|Stable version |✓|
|1.iii|Declared public API|✓|
|1.iv|API stability policy| --- |
|1.v|ABI stability policy| --- |
|1.vi_|API/ABI stable within ros distribution| --- |
|2| **Change control process** | --- |
|2.i| All changes occur on change request | ✓ |
|2.ii| Contributor origin (DCO, CLA, etc) | ✓ |
|2.iii| Peer review policy | ✓ |
|2.iv| CI policy for change requests | ✓ |
|2.v| Documentation policy for change requests | ✓ |
|3| **Documentation** | ✓ |
|3.i| Per feature documentation | ✓ |
|3.ii| Per public API item documentation | ✓ |
|3.iii| Declared License(s) | ✓ |
|3.iv| Copyright in source files| ✓ |
|3.v.a| Quality declaration linked to README | ✓ |
|3.v.b| Centralized declaration available for peer review | --- |
|4| Testing | --- |
|4.i| Feature items tests | ✓ |
|4.ii| Public API tests | ✓ |
|4.iii.a| Using coverage |✓ |
|4.iii.a| Coverage policy | --- |
|4.iv.a| Performance tests (if applicable) | --- |
|4.iv.b| Performance tests policy| --- |
|4.v.a| Code style enforcement (linters)| ✓ |
|4.v.b| Use of static analysis tools | ✓ |
|5| Dependencies | --- |
|5.i| Must not have ROS lower level dependencies | --- |
|5.ii| Optional ROS lower level dependencies| --- |
|5.iii| Justifies quality use of non-ROS dependencies | --- |
|6| Platform support | --- |
|6.i| Support targets Tier1 ROS platforms| --- |
|7| Security | --- |
|7.i| Vulnerability Disclosure Policy | --- |
