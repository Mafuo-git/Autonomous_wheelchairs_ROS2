# wheelchair-caninterface


wheelchair-caninterface package repository

# Adding a GitLab Repository as a Submodule to a ROS 2 Workspace

This guide explains how to add an existing GitLab repository as a Git submodule within your ROS 2 workspace repository, maintaining separate version control for both the workspace and the package.

## Prerequisites
- Existing ROS 2 workspace repository on GitLab
- Existing ROS 2 package repository on GitLab
- Git installed on your system
- Basic familiarity with Git commands

## Step-by-Step Procedure

### 1. Clone Your Workspace Repository (if not already cloned)
```bash
git clone git@gitlab.com:your-username/your-workspace-repo.git
cd your-workspace-repo
```

### 2. Add the Package as a Submodule
```bash
git submodule add git@gitlab.com:package-owner/package-repo.git src/package_name
```

Example:
```bash
git submodule add git@gitlab.com:my-team/navigation_pkg.git src/navigation
```

### 3. Initialize and Update Submodules
```bash
git submodule init
git submodule update
```

### 4. Commit the Submodule Addition
```bash
git add .gitmodules src/package_name
git commit -m "Add package_name as submodule"
git push origin main  # or your branch name
```

### 5. Build the Workspace with the New Package
```bash
colcon build --symlink-install
source install/setup.bash
```

## Working with Submodules

### Cloning a Repository with Submodules
When someone clones your workspace:
```bash
git clone --recurse-submodules git@gitlab.com:your-username/your-workspace-repo.git
```

If already cloned without submodules:
```bash
git submodule init
git submodule update
```

### Updating Submodules
To update a submodule to its latest commit:
```bash
cd src/package_name
git pull origin main
cd ../..
git add src/package_name
git commit -m "Update package_name submodule"
git push
```

### Changing Submodule Branch
To track a specific branch:
```bash
cd src/package_name
git checkout desired-branch
git pull
cd ../..
git config -f .gitmodules submodule.src/package_name.branch desired-branch
git add .gitmodules src/package_name
git commit -m "Set package_name submodule to track desired-branch"
git push
```

## Best Practices

1. **Document Submodules**: Add a README.md explaining:
   - Why submodules are used
   - How to initialize/update them
   - Package version compatibility

2. **Branch Management**: 
   - Consider pinning submodules to specific commits (not branches) for stability
   - Document compatible versions in your main repository

3. **CI/CD Considerations**:
   ```yaml
   # Example .gitlab-ci.yml configuration
   build:
     script:
       - git submodule sync --recursive
       - git submodule update --init --recursive
       - colcon build
   ```

4. **Alternative for Large Teams**:
   For complex projects, consider using:
   - [vcstool](https://github.com/dirk-thomas/vcstool) for package management
   - ROS 2's `repos` file format

## Troubleshooting

**Issue**: Submodule appears empty  
**Solution**: Run `git submodule update --init --recursive`

**Issue**: Permission denied when cloning  
**Solution**: Ensure SSH keys are set up properly for both repositories

**Issue**: Build fails after submodule update  
**Solution**: Check package compatibility and update dependencies accordingly

Save this document as `SUBMODULE_GUIDE.md` in your project for future reference.