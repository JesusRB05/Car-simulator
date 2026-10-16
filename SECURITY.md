# Security Policy

## Supported versions

Car Simulator is pre-1.0. Security fixes are made on the `main` branch only.

## Reporting a vulnerability

Please **do not** open a public issue, pull request or discussion for a
security problem.

Report it privately through GitHub Security Advisories:

1. Go to the repository's **Security** tab.
2. Click **Report a vulnerability**.
3. Describe the issue, the affected component (for example the mod loader,
   Lua sandbox, plugin API, asset/glTF loading or the build scripts), the
   steps to reproduce and the impact you expect.

Maintainers will acknowledge the report, work on a fix in a private advisory
and credit you when the advisory is published, unless you prefer to stay
anonymous.

## Scope notes

Mods and plugins are third-party code. Native plugins run with the full
privileges of the simulator, so they are off by default and must be switched on
and allowlisted by the user (`plugins.enabled` / `plugins.allow` in
`config.toml`); each library is fingerprinted with SHA-256 when it is found.
Only install plugins you trust. Weaknesses in the Lua sandbox, in the plugin
allowlist/hash gate or in the validation of mod manifests and assets are in
scope.
