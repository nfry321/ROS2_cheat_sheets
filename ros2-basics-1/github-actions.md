# Github actions

Github actions are relivitvly new addition to github and allow you to run automatic tests on code.

Some ROS specific tests have already been created so we should be able to use these fairly easily. 

{% embed url="https://ubuntu.com/blog/ros-2-ci-with-github-actions" %}

{% embed url="https://discourse.ros.org/t/github-actions-for-ros-and-ros-2-ci-beta/12755" %}

should we run on our docker setup to ensure it all works together?

{% embed url="https://github.com/marketplace/actions/setup-ros-environment\#alternative-to-setup-ros" %}

{% tabs %}
{% tab title="lint" %}
```yaml
name: Lint ROS2 package
on:
  # Run this action whenever a pull request is made
  pull_request:
  push:
    branches:
      - master

jobs:
  ament_lint:
    name: ament_${{ matrix.linter }}
    runs-on: ubuntu-20.04
    strategy:
      fail-fast: false
      matrix:
        linter: [copyright, xmllint, flake8, pep257]
        # generic linters [copyright, xmllint, lint_cmake]
        # python linters [flake8, pep257]
        # cpp linters [cppcheck, cpplint, uncrustify]
    steps:
      - uses: actions/checkout@v2.3.4
      - name: Setup ROS environment
        uses: ros-tooling/setup-ros@0.1.1
        with:
          required-ros-distributions: foxy

      - name: ROS 2 Lint Action
        uses: ros-tooling/action-ros-lint@0.1.0
        with:
          linter: ${{ matrix.linter }}
          package-name: fill_in_pkg_name
          distribution: foxy

```
{% endtab %}

{% tab title="build" %}
```yaml
name: Test
on:
  pull_request:
  push:
    branches:
      - master

jobs:
  build-and-test:
    runs-on: ubuntu-20.04
    container:
      image: ubuntu:focal
    steps:
    - name: Setup ROS environment
      uses: ros-tooling/setup-ros@0.1.1
      with:
        required-ros-distributions: foxy

    - name: ROS 2 CI Action
      uses: ros-tooling/action-ros-ci@0.1.1
      with:
        package-name: fill_in_pkg_name
        target-ros2-distro: foxy
        import-token: ${{ secrets.TEST_KEY }}

    - name: Upload a Build Artifact
      uses: actions/upload-artifact@v2
      with:
        name: colcon-logs
        path: ros_ws/log
      if: always()

```
{% endtab %}
{% endtabs %}

For private repos a [secret key is required](https://github.com/marketplace/actions/ros-2-ci-action#use-with-a-private-repo). 

