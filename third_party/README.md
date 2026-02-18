# Third-Party Dependencies

This directory contains third-party libraries used by the project.

## esmini

**Version**: v2.59.0  
**Source**: https://github.com/esmini/esmini  
**Download URL**: https://github.com/esmini/esmini/releases/download/v2.59.0/esmini-demo_Linux.zip

### Setup

The esmini library is **not included** in the repository. You must download it before building:

```bash
# From the project root
./scripts/setup_esmini.sh
```

This script will:
1. Download the esmini demo package from GitHub
2. Extract it to `third_party/esmini/`
3. Verify the installation

### What Gets Committed

Only the following files are tracked in Git:
- `BUILD` - Bazel build configuration
- `config.yml` - Configuration metadata
- `LICENSE` - License information
- `version.txt` - Version tracking
- `release_notes.md` - Release notes

All other esmini files (binaries, libraries, resources) are downloaded by the setup script and excluded via `.gitignore`.

### Manual Download

If you prefer to download manually:

1. Download: https://github.com/esmini/esmini/releases/download/v2.59.0/esmini-demo_Linux.zip
2. Extract to a temporary location
3. Rename the extracted `esmini-demo` folder to `esmini`
4. Move it to `third_party/esmini/`

### Updating esmini

To update to a new version:

1. Update the version and URL in `scripts/setup_esmini.sh`
2. Update `version.txt` in this directory
3. Test the build: `bazel build //...`
4. Update documentation if interface changes

## License

See individual LICENSE files in each third-party directory.
