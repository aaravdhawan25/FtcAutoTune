plugins {
    id("com.android.library") version "8.1.4" apply false
}

allprojects {
    group = property("GROUP") as String
    version = property("VERSION_NAME") as String
}
