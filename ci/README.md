<!--
    Copyright 2024, UNSW

    SPDX-License-Identifier: BSD-2-Clause
-->

# CI

## Building examples

Right now we only have build-time checks for all examples under `examples/`.

This is a check that runs on each commit and pull request made to the repository.
This check is required to pass for merging a pull request.

You can reproduce what the CI runs with:
```sh
./ci/examples.sh /path/to/sdk
```

You will need to provide the path to your Microkit SDK.

## Style

The CI runs a style check on any changed files and new files added in each GitHub
pull request.

We use the same style checking as the official seL4 projects. The style check primarily
exists not for enforcing a certain style, but for having some consistency with the code
in order to make it more readable.

You can reproduce the style check by doing the following:
```sh
git clone https://github.com/seL4/seL4_tools
./seL4_tools/misc/style-all.sh /path/to/what/to/style
```

Note that you will need to have the `astyle` program installed (e.g `apt install astyle`
or `brew install astyle`).

All the styling scripts can be found in `seL4_tools/misc`. You can run the script on an
entire directory or just on a particular file.

