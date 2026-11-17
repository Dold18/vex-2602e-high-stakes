# Publishing on GitHub (macOS)

The included CAD assembly is larger than GitHub's standard 100 MiB Git object limit. **Git LFS is required before `git add .`.** The `.gitattributes` file is already configured.

1. Create an empty PUBLIC repository named `vex-2602e-high-stakes` at https://github.com/new. Do NOT initialize it with a README, license, or .gitignore.
2. Extract the archive on your Mac. Open Terminal in the extracted `vex-2602e-high-stakes` folder.
3. If needed, install Homebrew from https://brew.sh/ and run `brew install git-lfs`.
4. Run the commands below from that folder.

```bash
git lfs install
git init -b main
git add .
git lfs ls-files           # Must show cad/2602e-worlds-robot.f3z
git commit -m "Initial VEX 2602E High Stakes engineering portfolio"
git remote add origin https://github.com/Dold18/vex-2602e-high-stakes.git
git push -u origin main
```

If `git lfs ls-files` shows no CAD file, stop before pushing and run `git lfs track 'cad/*.f3z'`, `git add .gitattributes cad/2602e-worlds-robot.f3z`, then check again.

Once pushed, open `https://github.com/Dold18/vex-2602e-high-stakes` and verify the README, PDF, images, video links and LFS-backed CAD.

**When code arrives:** Put the original project under `code/`, remove `code/.gitkeep` if desired, commit and push. No fabricated source code is included in this release.
