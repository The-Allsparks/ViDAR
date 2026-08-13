package android.content.res;

import java.io.IOException;
import java.io.InputStream;

/** Minimal stub for JVM unit tests. */
public final class AssetManager {
    public InputStream open(String path) throws IOException {
        throw new IOException("AssetManager stub — use loadSeason(String) in tests");
    }
}
