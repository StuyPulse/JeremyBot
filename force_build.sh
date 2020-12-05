#!/bin/sh

# https://github.com/StuyPulse/StuyLib/blob/main/force_build.sh

# IF YOU ARE RUNNING INTO ERRORS, THIS SCRIPT WILL RUN THROUGH 
# ALL THE STEPS NEEDED TO BUILD CORRECTLY, IF THIS DOES NOT WORK 
# THERE IS SOMETHING ELSE WRONG WITH YOUR PROJECT.

echo ""
echo "+------------------------+"
echo "| Formatting Document... |"
echo "+------------------------+"

# Apply formatter
#   - The build will not pass if the project is not formatted
gradle spotlessApply

echo ""
echo "+----------------------+"
echo "| Building Jar File... |"
echo "+----------------------+"

# Build Jar
gradle build