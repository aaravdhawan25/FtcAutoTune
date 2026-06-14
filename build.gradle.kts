plugins {
    id("com.android.library") version "8.7.0" apply false
}

allprojects {
    group = property("GROUP") as String
    version = property("VERSION_NAME") as String
}
