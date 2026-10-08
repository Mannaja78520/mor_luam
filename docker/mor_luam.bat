@echo off
rem mor_luam helper for Windows + Docker Desktop.  Usage:  docker\mor_luam.bat help
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0mor_luam.ps1" %*
