#if UNITY_EDITOR
using System;
using System.IO;
using UnityEditor;
using UnityEngine;

namespace UavUsv.Editor.Tools
{
    /// <summary>
    /// Isolated build adapter for the Vue frontend. It does not change the
    /// simulation source; it only prepares browser-compatible build settings.
    /// </summary>
    public static class VueWebGlBuildTool
    {
        private const string ScenePath = "Assets/Scenes/UavUsvDemo.unity";

        [MenuItem("UAV-USV/Tools/Build Vue WebGL")]
        public static void Build()
        {
            if (EditorApplication.isPlayingOrWillChangePlaymode)
                throw new UnityEditor.Build.BuildFailedException("Exit Play Mode before building Vue WebGL.");

            if (!File.Exists(ScenePath))
                throw new UnityEditor.Build.BuildFailedException(
                    "Unity scene is missing: " + ScenePath
                );

            // Build the checked-in/current scene as-is. Recreating it here
            // would discard scene-level configuration immediately before the
            // WebGL build and make the embedded player differ from the editor.
            AssetDatabase.SaveAssets();
            string requestedOutput = Environment.GetEnvironmentVariable("UAV_USV_WEBGL_OUTPUT");
            string outputPath = Path.GetFullPath(string.IsNullOrWhiteSpace(requestedOutput)
                ? Path.Combine(
                    Application.dataPath,
                    "..",
                    "..",
                    "UAV_USV_Platform",
                    "frontend",
                    "public",
                    "unity"
                )
                : requestedOutput);
            string integrationIndexPath = Path.Combine(outputPath, "index.html");
            string integrationIndex = File.Exists(integrationIndexPath)
                ? File.ReadAllText(integrationIndexPath)
                : string.Empty;

            Directory.CreateDirectory(outputPath);
            try
            {
                PlayerSettings.WebGL.compressionFormat = WebGLCompressionFormat.Disabled;
                PlayerSettings.WebGL.decompressionFallback = false;
                PlayerSettings.stripEngineCode = false;
                var report = BuildPipeline.BuildPlayer(new BuildPlayerOptions
                {
                    scenes = new[] { ScenePath },
                    locationPathName = outputPath,
                    target = BuildTarget.WebGL,
                    options = BuildOptions.None
                });

                if (report.summary.result != UnityEditor.Build.Reporting.BuildResult.Succeeded)
                    throw new UnityEditor.Build.BuildFailedException("Vue WebGL build failed: " + report.summary.result);
            }
            finally
            {
                if (!string.IsNullOrWhiteSpace(integrationIndex) &&
                    integrationIndex.Contains("source: 'unity-webgl'"))
                    File.WriteAllText(integrationIndexPath, integrationIndex);
            }

            Debug.Log("Vue WebGL build complete: " + outputPath);
        }
    }
}
#endif
