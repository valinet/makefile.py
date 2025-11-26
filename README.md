## makefile.py ##

Automatically generate a Makefile for a C/C++ codebase. Includes support for
[CCH](https://github.com/tjps/cch) files.

makefile.py is designed to be run each time you would run `make`.
It determines all relevant source files, parses them to recreate the
dependency graph, and outputs a Makefile that can then be run normally with `make`.

One goal of this project is to have minimal dependencies:

  * GNU Make
  * Python 3.x

Compared to the upstream version, this fork makes the followinf changes:

  * Adds headers as dependencies for each target. This was done because in my projects I
  often have structs (that go to the header file) whose members references lambdas which
  I change, and that did not trigger a recompile of the respective unit, even though I
  was making changes.
  * Support for compilation of projects using modules. In my projects, all external
  dependencies, including the standard library, are brought in as C++ modules, so this
  supports building the cache before performing the compilation of the main executable.
  * Support reading configuration from `src/config.h`.
  * Defaults to C23 and C++23 standards, O3 optimization, hardcoded source and build
  folders, flags and cross compilers for `armv7` (this can easily be changed in the file).
  * Added rules for debugging from Visual Studio Code, deployment, local emulation
  (using `qemu`), building documentation, building cch beforehand, creating output
  directories (when running `make`, not the generator)
  * Invoke `cch` with `--diff` (do not touch file if output is identical, i.e. the file
  has not changed) and `--noBanner` (do not show copyright banner on execution) options.
  * Removed support for [protobuf](https://developers.google.com/protocol-buffers) files,
  and [gRPC](https://grpc.io/) service definitions
  * Support for `auto main() -> int` syntax for `main`
