import { spawnSync } from "node:child_process";
import { existsSync } from "node:fs";

const inputPath = process.argv[2];
const outputPath = process.argv[3];

if (!inputPath || !outputPath || !existsSync(inputPath)) {
  process.exit(0);
}

function getCandidateExecutables() {
  if (process.platform === "win32") {
    const programFiles = process.env.PROGRAMFILES;
    const programFilesX86 = process.env["PROGRAMFILES(X86)"];
    const localAppData = process.env.LOCALAPPDATA;

    return [
      programFiles ? `${programFiles}\\draw.io\\draw.io.exe` : null,
      programFilesX86 ? `${programFilesX86}\\draw.io\\draw.io.exe` : null,
      localAppData ? `${localAppData}\\Programs\\draw.io\\draw.io.exe` : null,
      programFiles ? `${programFiles}\\draw.io\\diagrams.net.exe` : null,
      programFilesX86 ? `${programFilesX86}\\draw.io\\diagrams.net.exe` : null,
      localAppData
        ? `${localAppData}\\Programs\\diagrams.net\\diagrams.net.exe`
        : null,
      "draw.io",
      "diagrams.net",
    ].filter(Boolean);
  }

  if (process.platform === "darwin") {
    return [
      "/Applications/draw.io.app/Contents/MacOS/draw.io",
      "/Applications/diagrams.net.app/Contents/MacOS/diagrams.net",
      "draw.io",
      "diagrams.net",
    ];
  }

  return ["drawio", "draw.io", "diagrams.net"];
}

function runExport(executable) {
  const result = spawnSync(
    executable,
    ["--export", "--format", "png", "--output", outputPath, inputPath],
    {
      stdio: "ignore",
      windowsHide: true,
      shell: false,
    },
  );

  return result.status === 0;
}

for (const executable of getCandidateExecutables()) {
  const isAbsolutePath =
    executable.includes("/") || executable.includes("\\") || executable.includes(":");

  if (isAbsolutePath && !existsSync(executable)) {
    continue;
  }

  try {
    if (runExport(executable)) {
      process.exit(0);
    }
  } catch {
    // Ignore and try the next candidate executable.
  }
}

process.exit(0);
