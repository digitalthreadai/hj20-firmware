#!/usr/bin/env python3
"""Generate the HJ-20 firmware SBOM.

Outputs
  * an SPDX 2.3 JSON document (uploaded as the `sbom-spdx` CI artifact and
    attached to tagged releases), and
  * optionally a GitHub dependency-submission snapshot, so the repository's
    dependency graph (GET /repos/{owner}/{repo}/dependency-graph/sbom) lists
    the same components.

Inputs are sbom/components.json and the project version in CMakeLists.txt.
Standard library only.

Copyright (c) 2026 Helix Robotics (fictional demonstration company, DigitalEng.ai).
All rights reserved.
"""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
LICENSE_REF = "LicenseRef-Helix-Proprietary"


def project_version() -> str:
    text = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    m = re.search(r"project\(\s*hj20_firmware\s+VERSION\s+(\d+\.\d+\.\d+)", text)
    if not m:
        sys.exit("cannot find project VERSION in CMakeLists.txt")
    return m.group(1)


def files_for(paths: list[str]) -> list[pathlib.Path]:
    out: list[pathlib.Path] = []
    for p in paths:
        full = ROOT / p
        if full.is_dir():
            out.extend(sorted(f for f in full.rglob("*") if f.is_file()))
        elif full.is_file():
            out.append(full)
        else:
            sys.exit(f"component path not found: {p}")
    return out


def digest(files: list[pathlib.Path]) -> str:
    """SHA-256 over (relative path, content) of every source file, in order."""
    h = hashlib.sha256()
    for f in files:
        h.update(f.relative_to(ROOT).as_posix().encode())
        h.update(b"\0")
        h.update(f.read_bytes().replace(b"\r\n", b"\n"))
    return h.hexdigest()


def purl(name: str, version: str) -> str:
    return f"pkg:generic/helix-robotics/{name}@{version}"


def spdx_id(name: str) -> str:
    return "SPDXRef-Package-" + re.sub(r"[^A-Za-z0-9.-]", "-", name)


def build_spdx(manifest: dict, version: str, sha: str, toolchain: str) -> dict:
    now = dt.datetime.now(dt.timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")
    supplier = f"Organization: {manifest['supplier']}"
    root_id = spdx_id(manifest["product"])

    packages = [{
        "SPDXID": root_id,
        "name": manifest["product"],
        "versionInfo": version,
        "supplier": supplier,
        "downloadLocation": "NOASSERTION",
        "filesAnalyzed": False,
        "licenseConcluded": LICENSE_REF,
        "licenseDeclared": LICENSE_REF,
        "copyrightText": manifest["copyright"],
        "externalRefs": [{
            "referenceCategory": "PACKAGE-MANAGER",
            "referenceType": "purl",
            "referenceLocator": purl(manifest["product"], version),
        }],
    }]
    relationships = [{
        "spdxElementId": "SPDXRef-DOCUMENT",
        "relationshipType": "DESCRIBES",
        "relatedSpdxElement": root_id,
    }]

    for comp in manifest["components"]:
        cid = spdx_id(comp["name"])
        packages.append({
            "SPDXID": cid,
            "name": comp["name"],
            "versionInfo": version,
            "supplier": supplier,
            "downloadLocation": "NOASSERTION",
            "filesAnalyzed": False,
            "checksums": [{"algorithm": "SHA256", "checksumValue": digest(files_for(comp["paths"]))}],
            "licenseConcluded": LICENSE_REF,
            "licenseDeclared": LICENSE_REF,
            "copyrightText": manifest["copyright"],
            "description": comp["description"],
            "externalRefs": [{
                "referenceCategory": "PACKAGE-MANAGER",
                "referenceType": "purl",
                "referenceLocator": purl(comp["name"], version),
            }],
        })
        relationships.append({
            "spdxElementId": root_id,
            "relationshipType": "CONTAINS",
            "relatedSpdxElement": cid,
        })

    return {
        "spdxVersion": "SPDX-2.3",
        "dataLicense": "CC0-1.0",
        "SPDXID": "SPDXRef-DOCUMENT",
        "name": f"{manifest['product']}-{version}",
        "documentNamespace": f"https://spdx.helix-robotics.example/{manifest['product']}/{version}/{sha or 'local'}",
        "creationInfo": {
            "created": now,
            "creators": ["Tool: hj20-gen-sbom-1.0", supplier],
            "comment": f"Commit {sha or 'local'}. Build tool (not linked): {toolchain}. "
                       + " ".join(manifest.get("notes", [])),
        },
        "packages": packages,
        "relationships": relationships,
        "hasExtractedLicensingInfos": [{
            "licenseId": LICENSE_REF,
            "name": "Helix Robotics proprietary license",
            "extractedText": manifest["copyright"],
        }],
    }


def build_snapshot(manifest: dict, version: str, sha: str, ref: str, run_id: str, repo_url: str) -> dict:
    now = dt.datetime.now(dt.timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")
    resolved = {
        comp["name"]: {
            "package_url": purl(comp["name"], version),
            "relationship": "direct",
            "scope": "runtime",
        }
        for comp in manifest["components"]
    }
    return {
        "version": 0,
        "sha": sha,
        "ref": ref,
        "job": {"correlator": "ci-sbom", "id": run_id or "local"},
        "detector": {"name": "hj20-gen-sbom", "version": "1.0", "url": repo_url or "https://example.invalid"},
        "scanned": now,
        "manifests": {
            "sbom/components.json": {
                "name": "sbom/components.json",
                "file": {"source_location": "sbom/components.json"},
                "resolved": resolved,
            }
        },
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", required=True, help="SPDX JSON output path")
    ap.add_argument("--snapshot", help="dependency-submission snapshot output path")
    ap.add_argument("--sha", default="", help="commit SHA (GITHUB_SHA)")
    ap.add_argument("--ref", default="", help="git ref (GITHUB_REF)")
    ap.add_argument("--run-id", default="", help="CI run id")
    ap.add_argument("--repo-url", default="", help="repository URL")
    ap.add_argument("--toolchain", default="arm-none-eabi-gcc (version not recorded)")
    args = ap.parse_args()

    manifest = json.loads((ROOT / "sbom" / "components.json").read_text(encoding="utf-8"))
    version = project_version()

    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(build_spdx(manifest, version, args.sha, args.toolchain), indent=2) + "\n",
                   encoding="utf-8")
    print(f"wrote {out} ({len(manifest['components'])} components, version {version})")

    if args.snapshot:
        if not args.sha or not args.ref:
            sys.exit("--snapshot needs --sha and --ref")
        snap = pathlib.Path(args.snapshot)
        snap.parent.mkdir(parents=True, exist_ok=True)
        snap.write_text(json.dumps(build_snapshot(manifest, version, args.sha, args.ref,
                                                  args.run_id, args.repo_url), indent=2) + "\n",
                        encoding="utf-8")
        print(f"wrote {snap}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
