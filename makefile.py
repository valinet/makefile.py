#!/usr/bin/env python3
# -- https://github.com/tjps/makefile.py --
# This script is an automatic Makefile generator for C/C++ codebases.
#
# It finds all buildable object file sources (e.g. .cch, .cc, ...)
# and generates a build command for each one based on the
# list of #include's it contains.
#
# For each .cc that contains "int main(...)" a link step is generated.
#
# The end result is a Makefile that has a build target for each executable
# with all of the compile time and link time dependencies in place.
#
# There are additional nuances, such as special comments that can inject
# compile or link time flags, and debug output to check the dependency
# graph for cycles.  TODO: this last piece is no longer true.
#
# Compile/link arguments for an object can be specified
# with the special comment forms:
#
#     // @compileargs -Ithird-party/libfoo/include/ (etc.)
#     // @linkargs -lz (etc.)
#


import os
import re
import sys
import pathlib
import argparse
import re
import subprocess
from enum import Enum
from textwrap import dedent
from collections import defaultdict
from os.path import join as pathjoin
from datetime import datetime, timezone

def get_clean_synced_commit():
    def run(cmd, check=False):
        return subprocess.run(cmd, text=True, capture_output=True, check=check)
    # Get current HEAD
    try:
        head = run(["git", "rev-parse", "HEAD"], check=True).stdout.strip()
    except subprocess.CalledProcessError:
        return ""
    # Detect unstaged changes
    if run(["git", "diff", "--quiet"]).returncode != 0:
        return ""
    # Detect staged changes
    if run(["git", "diff", "--cached", "--quiet"]).returncode != 0:
        return ""
    # Detect untracked files
    untracked = run(["git", "ls-files", "--others", "--exclude-standard"])
    if untracked.stdout.strip():
        return ""
    # Fetch
    if run(["git", "fetch", "origin"]).returncode != 0:
        return ""
    # Get upstream commit
    try:
        upstream = run(["git", "rev-parse", "origin/master"], check=True).stdout.strip()
    except subprocess.CalledProcessError:
        return ""
    # Compare HEAD to upstream
    if head != upstream:
        return ""
    return head

defines = {}

file_types = {}
def handles(extension, produces=None):
    """ Class decorator that registers a class
        as handling a specific file extension. """
    def deco(cls):
        file_types[extension] = cls
        setattr(cls, "extension", extension)
        setattr(cls, "produces", produces or [])
        return cls
    return deco


# These regex helpers remain at global scope so that the
# compiled regex caching is done once per process.
def has_main(contents, regex=re.compile(r'^\s*(?:int|auto)\s+main\s*\((int[^\)]*|\s*)\)', re.MULTILINE)):
    return regex.search(contents) != None
def get_includes(contents, regex=re.compile(r'^\s*#include\s*"([^"]+)"\s*$', re.MULTILINE)):
    include_regex = re.compile(r'^\s*#include\s*"([^"]+)"')
    lines = contents.splitlines()
    output_includes = []
    i = 0
    n = len(lines)
    skipping = False  # inside a skipped #if block
    skip_depth = 0    # nested #if levels while skipping
    while i < n:
        line = lines[i].rstrip()
        if not (line.strip() == "" or
                line.strip().startswith("//") or
                line.strip().startswith("#") or
                line.strip().startswith("/**")):
            break
        if line.strip().startswith("//"):
            i += 1
            continue
        if line.strip().startswith("#"):
            m = re.match(r'^\s*#if\s+defined\(([^)]+)\)', line)
            if m:
                symbol = m.group(1)
                if skipping:
                    skip_depth += 1
                else:
                    if symbol not in defines:
                        skipping = True
                        skip_depth = 1
                i += 1
                continue
            if re.match(r'^\s*#endif', line):
                if skipping:
                    skip_depth -= 1
                    if skip_depth == 0:
                        skipping = False
                i += 1
                continue
            if skipping:
                i += 1
                continue
            inc = include_regex.findall(line)
            if inc:
                output_includes.extend(inc)
        i += 1
    return output_includes
#    return regex.findall(contents)
def get_imports(contents, regex=re.compile(r'^\s*import\s+(?:"([^"]+)"|([^;\s]+))\s*;\s*$', re.MULTILINE)):
    return [ m.group(1) or m.group(2) for m in regex.finditer(contents) ]
def get_compileargs(contents, regex=re.compile(r'^\s*//\s*@compileargs\s+(.*)\s*$', re.MULTILINE)):
    return regex.findall(contents)
def get_linkargs(contents, regex=re.compile(r'^\s*//\s*@linkargs\s+(.*)\s*$', re.MULTILINE)):
    return regex.findall(contents)



class Emitted:
    """ Struct returned by emit() describing emitted artifacts. """
    def __init__(self, directories=None, executables=None, patterns=None):
        self.directories = directories
        self.executables = executables
        self.patterns = patterns

class FileAction(Enum):
    INCOMPATIBLE = 1
    REPLACE = 2
    DROP = 3


class File:

    def __init__(self, filename, contents):
        self.fullpath = filename
        assert filename.startswith(src_dir)
        self.filename = filename[len(src_dir):]
        self.dirname = os.path.dirname(self.filename)
        (self.base, self.extension) = os.path.splitext(self.filename)
        self.dependencies = []
        self.includes = []
        self.compileargs = []
        self.linkargs = []
        self.imports = []
        self.is_link_target = False
        self.initialize(contents)

    def initialize(self, contents):
        assert False, "Not implemented in base class"

    def swap_extension(self, new_extension):
        return self.base + new_extension

    def get_variants(self, extensions, prepend_dir=""):
        return [pathjoin(prepend_dir, self.swap_extension(e)) for e in extensions]

    def get_aliases(self):
        return [self.filename]

    def artifacts(self):
        return self.get_variants(self.produces)

    def emit(self, out_dir):
        return Emitted()  # By default emit nothing.

    def has_relation(self, file):
        return FileAction.INCOMPATIBLE

    def __recursive_deps(self, deps=None, seen=None):
        # TODO: memoize?
        if not seen:
            seen = set()
        if not deps:
            deps = []
        for dep in self.dependencies:
            if not dep.filename in seen:
                seen.add(dep.filename)
                deps.append(dep)
                dep.__recursive_deps(deps, seen)
        return deps

    def __apply_rec(self, getter):
        ret = []
        for dep in self.__recursive_deps():
            ret += getter(dep)
        return ret

    def get_compile_dependencies(self):
        return self.__apply_rec(lambda dep: dep.get_aliases()[-1:])

    def get_link_dependencies(self):
        return self.__apply_rec(lambda dep: dep.artifacts())

    def get_linkargs(self):
        return set(self.__apply_rec(lambda dep: dep.linkargs))

    def __str__(self):
        return f"{self.filename}"


@handles(extension=".cc", produces=[".o"])
class CCFile(File):

    def initialize(self, contents):
        self.includes = get_includes(contents)
        self.compileargs = get_compileargs(contents)
        self.linkargs = get_linkargs(contents)
        self.is_link_target = has_main(contents)

    def get_aliases(self):
        return [self.filename, self.swap_extension(".h")]

    def has_relation(self, file):
        if self.swap_extension(".h") == file.filename:
            return FileAction.DROP
        return FileAction.INCOMPATIBLE

    def emit(self, out_dir):
        obj_file = pathjoin(out_dir, self.swap_extension(".o"))
        includes = " ".join([pathjoin(out_dir, f) for f in self.get_compile_dependencies()])
        compileargs = " ".join(self.compileargs)
        print(f"{obj_file}: {self.fullpath} {includes}")
        print(f"\t@$(ECHO) Building CXX object $@")
        print(f"\t@$(CXX) $(CXXFLAGS) {compileargs} -I{out_dir} -I{src_dir} -c $< -o $@")
        print()
        emitted = Emitted(directories=[os.path.dirname(obj_file)])
        if self.is_link_target:
            executable = pathjoin(out_dir, self.swap_extension(""))
            deps = " ".join([pathjoin(out_dir, x) for x in self.get_link_dependencies()])
            #linkargs = " ".join(list(self.get_linkargs()) + self.linkargs)
            print(f"{executable}: {deps} {obj_file}")
            print(f"\t@$(ECHO) Linking CXX executable $@")
            print(f"\t@$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $^ -pthread")
            print()
            emitted.executables = [executable]
        return emitted

@handles(extension=".c", produces=[".o"])
class CFile(File):

    def initialize(self, contents):
        self.includes = get_includes(contents)
        self.compileargs = get_compileargs(contents)
        self.linkargs = get_linkargs(contents)

    def get_aliases(self):
        return [self.filename, self.swap_extension(".h")]

    def has_relation(self, file):
        if self.swap_extension(".h") == file.filename:
            return FileAction.DROP
        return FileAction.INCOMPATIBLE

    def emit(self, out_dir):
        obj_file = pathjoin(out_dir, self.swap_extension(".o"))
        includes = " ".join([pathjoin(out_dir, f) for f in self.get_compile_dependencies()])
        compileargs = " ".join(self.compileargs)
        print(f"{obj_file}: {self.fullpath} {includes}")
        print(f"\t@$(ECHO) Building C object $@")
        print(f"\t$(CC) $(CFLAGS) -I{src_dir} -I{out_dir} {compileargs} -c $< -o $@")
        print()
        return Emitted(directories=[os.path.dirname(obj_file)])

@handles(extension=".h")
class HeaderFile(File):

    def initialize(self, contents):
        self.includes = filter(lambda s: s not in self.get_aliases(), get_includes(contents))
        self.imports = get_imports(contents)
        self.linkargs = get_linkargs(contents)

    def has_relation(self, file):
        if self.base == file.base and file.extension in [CCFile.extension, CFile.extension]:
            return FileAction.REPLACE
        return FileAction.INCOMPATIBLE

    def emit(self, out_dir):
        import_deps = " ".join(["gcm.cache/" + (f",/{args.src_root}" + x if "/" in x else x) + ".gcm" for x in self.imports if x and x != "std"])
        build_dir = pathjoin(out_dir, self.dirname)
        dst = pathjoin(build_dir, "%.h" if len(import_deps) == 0 else f"{os.path.basename(self.filename)}")
        src = pathjoin(src_dir, self.dirname, "%.h" if len(import_deps) == 0 else f"{os.path.basename(self.filename)}")

        pattern = f"{dst}: {src} {import_deps}\n" \
                  f"\t@$(ECHO) Writing header $@\n" \
                  f"\t@mkdir -p {build_dir}\n" \
                  f"\t@cp $< $@\n"
        
        return Emitted(directories=[build_dir], patterns=[pattern])


@handles(extension=".cch", produces=[".cch.o"])
class CCHFile(File):

    def initialize(self, contents):
        self.includes = filter(lambda s: not s in self.get_aliases(), get_includes(contents))
        self.compileargs = get_compileargs(contents)
        self.linkargs = get_linkargs(contents)
        self.is_link_target = has_main(contents)

    def get_aliases(self):
        return [self.filename, self.filename + ".h"]

    def emit(self, out_dir):
        (cc_file, obj_file, header_file) = self.get_variants([".cch.cc", ".cch.o", ".cch.h"],
                                                prepend_dir=out_dir)
        includes = " ".join([pathjoin(out_dir, f) for f in self.get_compile_dependencies()])
        compileargs = " ".join(self.compileargs)

        build_dir = pathjoin(out_dir, self.dirname)
        output_base = pathjoin(build_dir, "%.cch")
        src_pattern = pathjoin(src_dir, self.dirname, "%.cch")
        include_path = pathjoin(self.dirname, "%f")
        
        if obj_file.endswith("build/base/pch.cch.o"):
            pattern = f"build/base/pch.cch.h.gch: src/base/pch.cch {includes}\n" \
                      f"\t@$(ECHO) Precompiling header $@\n" \
                      f"\t$(CXX) $(CXXFLAGS) -I{src_dir} -I{out_dir} -c -x c++-header $< -o $@\n"
            return Emitted(directories=[build_dir], patterns=[pattern])

        pattern = f"{output_base}.cc {output_base}.h: {src_pattern} | cch/build/cch\n" \
                  f"\t@$(ECHO) Building CCH object $@\n" \
                  f"\t$(CCH) --diff --noBanner --input $< --output={build_dir}/%f\n" # --pch \"base/pch.cch.h\"

        print(f"{obj_file}: {cc_file} {header_file} {includes}")
        print(f"\t@$(ECHO) Building CXX object $@")
        print(f"\t@$(CXX) $(CXXFLAGS) -I{out_dir} -I{src_dir} {compileargs} -c $< -o $@")
        print()
        emitted = Emitted(directories=[build_dir], patterns=[pattern])
        if self.is_link_target:
            executable = pathjoin(out_dir, self.swap_extension(""))
            deps = " ".join([pathjoin(out_dir, x) for x in self.get_link_dependencies() if x != "base/pch.cch.o"])
            #linkargs = " ".join(list(self.get_linkargs()) + self.linkargs)
            print(f"{executable}: {deps} {obj_file} | cch/build/cch")
            print(f"\t@$(ECHO) Linking CXX executable $@")
            print(f"\t@$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $^ -pthread")
            print()
            emitted.executables = [executable]
        return emitted


def find_files(root_dir, extensions):
    """ Return all files under a subdirectory
        that match the extensions filter. """
    for (dirname, subdirs, files) in os.walk(root_dir, topdown=False):
        # Yield each file that matches the extensions list.
        for filename in files:
            (_, extension) = os.path.splitext(filename)
            if extension in extensions:
                yield pathjoin(dirname, filename)
        # Recurse into each subdirectory.
        for subdir in subdirs:
            find_files(pathjoin(root_dir, subdir), extensions)



if __name__ == "__main__":
    conffile = "src/config.h"

    pattern = re.compile(
        r'^\s*#define\s+([A-Za-z_][A-Za-z0-9_]*)'   # macro name
        r'(?:\s+(.*?))?'                            # optional value
        r'\s*(?://.*)?$',                           # optional comment
        re.MULTILINE
    )

    with open(conffile) as f:
        text = f.read()

    for name, value in pattern.findall(text):
        # Clean value: strip whitespace
        value = re.sub(r'^(["\'])(.*)\1$', r'\2', value).strip() if value else None
        defines[name] = value

    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    #parser.add_argument("src_root", help="Base directory of source tree")
    #parser.add_argument("build_root", help="Base directory of build output")
    parser.add_argument("--std", default="c++23", help="C++ version")
    parser.add_argument("--cstd", default="c23", help="C version")
    parser.add_argument("--optimization", default="2", help="Optimization level (-OX)")
    parser.add_argument("--debug", action="store_true", help="Debug logging to stderr")
    args = parser.parse_args()
    args.src_root = "src/"
    args.build_root = "build"
    args.modules_dir = "base/modules"
    args.do_not_emit_unused=True

    # Set the two global variables.
    globals()["debug"] = lambda s: print(f">> {s}", file=sys.stderr) #if args.debug else None
    src_dir = args.src_root

    _proj_repo = subprocess.check_output("basename -s .git `git config --get remote.origin.url`", shell=True, text=True).strip()
    _proj_commit = get_clean_synced_commit()
    _proj_compiledon = datetime.now().astimezone().replace(microsecond=0).isoformat()
    _proj_hostname = subprocess.check_output("hostname", shell=True, text=True).strip()

    cflags = f" -D_FILE_OFFSET_BITS=64 -D_TIME_BITS=64 -D_PROJ_REPO=\\\"{_proj_repo}\\\" -D_PROJ_COMMIT=\\\"{_proj_commit}\\\" -D_PROJ_COMPILEDON=\\\"{_proj_compiledon}\\\" -D_PROJ_HOSTNAME=\\\"{_proj_hostname}\\\"" \
             f" -march=armv7-a -marm -mfpu=neon -mfloat-abi=hard -O{args.optimization}" \
             f" -Wall -Wextra -Werror -Wno-psabi" \
             f" -fno-PIE -fno-omit-frame-pointer -fstack-protector-strong" \
             f" -pedantic -fdiagnostics-color=always -Wl,--gc-sections"
    cxxflags = f"-std={args.std}{cflags} -fno-exceptions -fno-rtti -Werror=unused-parameter -ffunction-sections -fdata-sections -fmodules"
    ldflags = f"-L../../../build/software/target/usr/lib"
    cflags = f"-std={args.cstd}{cflags}"

    # Print the normal Makefile pre-amble - setting of tool names, flags, etc.
    # The default target is 'all', which is a list of all linkable executables.
    # Also provides a 'clean' target which removes the build directory.
    # -Wno-unused-parameter -Wno-sign-compare
    print(dedent(f"""\
    BUILD_START := $(shell date +%s%N)
    CC = ../../../build/software/host/bin/arm-none-linux-gnueabihf-gcc
    CXX =../../../build/software/host/bin/arm-none-linux-gnueabihf-g++
    CCH ?= cch/build/cch
    CFLAGS = {cflags}
    CXXFLAGS = {cxxflags}

    _mkfile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
    I := $(patsubst %/,%,$(dir $(_mkfile_path)))
    ifneq ($(words $(MAKECMDGOALS)),1) # if no argument was given to make...
    .DEFAULT_GOAL = all # set the default goal to all
    %:                   # define a last resort default rule
    \t@$(MAKE) $@ --no-print-directory -rRf $(firstword $(MAKEFILE_LIST)) # recursive make call, 
    else
    ifndef ECHO
    T := $(shell $(MAKE) $(MAKECMDGOALS) --no-print-directory \
      -nrRf $(firstword $(MAKEFILE_LIST)) \
      ECHO="COUNTTHIS" | grep -c "COUNTTHIS")
    N := x
    C = $(words $N)$(eval N := x $N)
    ECHO = C=$$(echo $(C)); \
       echo -ne "\r\033[1;36m[$$(printf '%3d' $${{C}})\057$$(printf '%3d' $T) $$(printf '%3d' $$(expr $${{C}} '*' 100 / $T))%]\033[0m"
    endif

    .PHONY: default
    default: all

    FORCE: ;
    
    .PHONY: clean
    clean: ## clear all build artifacts
    \trm -rf {args.build_root}
    \trm -rf gcm.cache
    \trm -rf cch/build

    help: # with thanks to Ben Rady and Jeremy Rifkin ## displays this information
    \t@grep -E '^[0-9a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {{FS = ":.*?## "}}; {{printf "\\033[36m%-20s\\033[0m %s\\n", $$1, $$2}}'
    """))

    files = []
    file_map = {}
    usage_count = {}

    # Enumerate the set of files in the source root that
    # we are interested in putting into the Makefile.
    for filename in find_files(src_dir, file_types.keys()):
        if args.do_not_emit_unused:
            usage_count[filename] = 0

        with open(filename, "r") as f:
            contents = f.read()

        (_, extension) = os.path.splitext(filename)
        file_class = file_types.get(extension)
        file = file_class(filename, contents)
        files.append(file)

        for alias in file.get_aliases():
            in_map = file_map.get(alias, None)
            if in_map:
                action = in_map.has_relation(file)
                if action == FileAction.INCOMPATIBLE:
                    raise Exception(f"{alias} ({type(in_map).__name__}) already exists"
                                    f"in file map as {in_map} ({type(in_map).__name__})")
                elif action == FileAction.DROP:
                    continue
                # Otherwise the action is REPLACE, so drop through and replace it.

            file_map[alias] = file

    # Resolve the dependency tree.
    for file in files:
        for include in file.includes:
            if include not in file_map:
                raise Exception(f"{file.filename} references unknown file {include}")
            include = file_map[include]
            if args.do_not_emit_unused:
                if args.src_root + include.filename in usage_count:
                    usage_count[args.src_root + include.filename] += 1
            # Don't add a file as a dependency of itself.
            if include is not file:
                file.dependencies.append(include)

    if args.do_not_emit_unused:
        unused_files = {k: v for k, v in usage_count.items() if v == 0}
        files = [ file for file in files if file.is_link_target or file.fullpath not in unused_files ]
        all_imports = []
        for file in files:
            for item in file.imports:
                if item not in all_imports:
                    all_imports.append(item)
        all_imports = ["gcm.cache/" + (f",/{args.src_root}" + x if "/" in x else x) + ".gcm" for x in all_imports if x]
    ldflags += " " + " ".join(" ".join(file.linkargs) for file in files if file.linkargs)

    executables = []
    build_directories = set()
    patterns = set()
    dir_targets = defaultdict(list)

    # Loop over the files and emit their Makefile stanzas.
    # This is done separately because the first time a file
    # is encountered in the initial loop above, it is unlikely
    # to have a full picture of its recursive dependencies.
    for file in files:
        cd = file.get_compile_dependencies()
        ld = file.get_link_dependencies()
        assert len(cd) == len(set(cd)), f"cd: {cd}"
        assert len(ld) == len(set(ld)), f"ld: {ld}"

        emitted = file.emit(args.build_root)
        assert isinstance(emitted, Emitted), \
            "emit() must return an Emitted object"
        if emitted.executables:
            executables += emitted.executables
            for executable in emitted.executables:
                dir_targets[os.path.dirname(executable)].append(executable)
        if emitted.directories:
            #for p in emitted.directories:
            #    os.makedirs(p, exist_ok=True)
            build_directories.update(emitted.directories)
        if emitted.patterns:
            patterns.update(emitted.patterns)

    for pattern in patterns:
        print(pattern)

    # Print a list of convenience directory build targets
    # that depend on all of the executables in that directory.
    for (dir, products) in dir_targets.items():
        print(f"{dir} {dir}/: {' '.join(products)}")
        print()

    for filename in find_files(args.src_root + args.modules_dir, [".cppm"]):
        module_name = f"{os.path.basename(filename).rsplit(".", 1)[0]}"
        if args.do_not_emit_unused:
            if f"gcm.cache/{module_name}.gcm" not in all_imports:
                continue
        module_deps = ""
        with open(filename, "r") as f:
            contents = f.read()
            module_deps = " ".join(["gcm.cache/" + x + ".gcm" for x in get_imports(contents) if x and x != "std"])
        print(f"gcm.cache/{module_name}.gcm: {args.src_root}{args.modules_dir}/{module_name}.cppm gcm.cache/std.gcm {module_deps}\n" \
                f"\t@$(ECHO) Compiling named module $@ and its object file\n" \
                f"\t@$(CXX) $(CXXFLAGS) -I{args.src_root} -fsearch-include-path -c $< -o $@.o\n\n")
        ldflags += f" gcm.cache/{module_name}.gcm.o"

    # Print the Makefile post-amble.  Include all of the
    # executable targets in the default Makefile target.
    # Output a target to make the directory structure in
    # the build directory.
    #build_directories = " \\\n    \t".join(build_directories)
    build_directories = " ".join(build_directories)
    executables = " \\\n    \t".join(executables)
    print(dedent(f"""\
    gcm.cache/,/{args.src_root}{args.modules_dir}/%.hm.gcm: {args.src_root}{args.modules_dir}/%.hm gcm.cache/std.gcm
    \t@$(ECHO) Compiling header unit $@
    \t@$(CXX) $(CXXFLAGS) -I{args.src_root} -fsearch-include-path -fmodule-header -x c++-header -c $<

    gcm.cache/std.gcm:
    \t@$(ECHO) Compiling standard library module $@
    \t@$(CXX) $(CXXFLAGS) -fsearch-include-path -fmodule-only -c bits/std.cc

    gcm.cache/std.compat.gcm: gcm.cache/std.gcm
    \t@$(ECHO) Compiling standard compat library module $@
    \t@$(CXX) $(CXXFLAGS) -fsearch-include-path -fmodule-only -c bits/std.compat.cc
    
    .PHONY: make
    make: ## regenerate this makefile using makefile.py
    \tpython3 make/makefile.py | tee Makefile 2>&1

    guard: ## generate a unique header guard identifier
    \t@echo "zguard_`(uuidgen | tr -d '-')`"
                 
    docs: Doxyfile ## generate documentation (requires doxygen)
    \tdoxygen Doxyfile

    emulate: build/{defines['MAKE_EXECUTABLE']} ## run executable in armhf emulator (requires qemu-arm-static)
    \tqemu-arm-static -L ../../../build/software/host/arm-none-linux-gnueabihf/sysroot build/{defines['MAKE_EXECUTABLE']}

    deploy: build/{defines['MAKE_EXECUTABLE']} ## deploy executable to target board over the network (using the IP specified in config.h)
    \tssh root@{defines['MAKE_DEPLOY_IP']} killall {defines['MAKE_EXECUTABLE']} gdbserver || true 2>&1
    \tscp build/{defines['MAKE_EXECUTABLE']} root@{defines['MAKE_DEPLOY_IP']}:/root/{defines['MAKE_EXECUTABLE']}

    vscode_remote_gdbserver: deploy
    \tsystemctl --user stop vscode_remote_gdbserver || true
    \tsystemctl --user reset-failed vscode_remote_gdbserver || true
    \tsystemd-run --user --unit=vscode_remote_gdbserver /bin/bash -c "ssh -L20001:127.0.0.1:20001 root@{defines['MAKE_DEPLOY_IP']} 'gdbserver :20001 /root/{defines['MAKE_EXECUTABLE']}'"
    \twt.exe ssh root@{defines['MAKE_DEPLOY_IP']} "pidof {defines['MAKE_EXECUTABLE']} | xargs -I {{}} cat /proc/{{}}/fd/1"
    
    debug: deploy ## debugs program on target board
    \tssh root@{defines['MAKE_DEPLOY_IP']} "gdbserver :20001 /root/{defines['MAKE_EXECUTABLE']}" &
    \tsleep 1
    \t../../../build/software/host/bin/arm-none-linux-gnueabihf-gdb -ex "file build/{defines['MAKE_EXECUTABLE']}" -ex "target remote {defines['MAKE_DEPLOY_IP']}:20001"
    
    cch/build/cch: FORCE
    \t$(MAKE) CXX=g++ -C cch -j`nproc`

    .PHONY: build-dirs
    build-dirs: FORCE
    \tmkdir -p {build_directories}

    .PHONY: all
    all: {executables} ## build program based on configuration in config.h
    \t@{{ \\
    \t    end=$$(date +%s%N); \\
    \t    diff_ns=$$((end - $(BUILD_START))); \\
    \t    diff_ms=$$((diff_ns / 1000000)); \\
    \t    ms=$$((diff_ms % 1000)); \\
    \t    sec=$$((diff_ms / 1000)); \\
    \t    min=$$((sec / 60)); \\
    \t    sec=$$((sec % 60)); \\
    \t    $(ECHO) "All done in "; \\
    \t    printf "\033[F\033[28C%dm%02d,%03ds.\\n" $$min $$sec $$ms; \\
    \t}}

    LDFLAGS = {ldflags}
    
    endif
    """));

    # Make sure the directory structure in the build directory
    # is in place.  This helps tools/compilers that won't build
    # directory structure on their own.
    #for build_dir in build_directories:
    #    pathlib.Path(build_dir).mkdir(parents=True, exist_ok=True)
