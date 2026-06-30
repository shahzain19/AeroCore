# Contributing

Thanks for contributing to AeroCore.

## Development Setup

```bash
mkdir -p build
cd build
cmake .. -DAEROCORE_BUILD_TESTS=ON
cmake --build . -j"$(nproc)"
ctest --output-on-failure
```

## Guidelines

- Keep changes focused and reviewable.
- Prefer small, isolated pull requests.
- Update docs when behavior or interfaces change.
- Add or update tests when the change affects core logic or regression risk.
- Preserve existing code style and keep comments concise.

## Pull Requests

Before opening a PR, please:

1. build successfully,
2. run the test suite,
3. summarize the user-visible or architectural impact,
4. mention any limitations or follow-up work.

## Reporting Bugs

When filing a bug, include:

- expected behavior,
- actual behavior,
- steps to reproduce,
- config used,
- platform/compiler details if relevant.
