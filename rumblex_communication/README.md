# Speech cache

TTS audio and metadata are saved in `rumblex_communication/tts_cache/`,
next to `soundfiles/`, using the same package-relative path resolver. The
directory is created automatically and does not depend on the working directory
or `XDG_CACHE_HOME`. With a source or symlink installation this is the source
package directory; a copied installation uses the installed package location,
which must be writable. Existing files in the previous user cache are not moved.

Filenames use the full SHA-256 hash of the language,
a NUL separator, and the exact UTF-8 text, so sentence length does not affect
filename length.

Each audio file has a readable UTF-8 JSON sidecar: `<hash>.mp3.json` or
`<hash>.wav.json`. It records the exact text, language, engine, audio filename,
and the eSpeak voice for fallback recordings. Separate sidecars keep fallback
WAVs identifiable even after an online MP3 is generated. Writes are atomic;
metadata is also refreshed when cached MP3 audio is used.

From the repository root, find a recording by spoken text:

```bash
rg -l -i 'high five' rumblex_communication/tts_cache --glob '*.json'
```

Read the matching JSON's `audio_file` field to locate the audio in the same
directory. Delete both the audio and its sidecar to remove a recording.
Online synthesis uses gTTS; on failure, eSpeak NG uses the language with the
`+f3` voice variant. Online synthesis is retried on subsequent requests when
only a fallback WAV exists.
