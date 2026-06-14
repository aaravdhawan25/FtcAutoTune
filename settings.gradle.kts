pluginManagement {
    repositories {
        google()
        mavenCentral()
        gradlePluginPortal()
    }
}

dependencyResolutionManagement {
    repositories {
        google()
        mavenCentral()
        maven { url = uri("https://maven.brott.dev/") } // FTC Dashboard, if used
    }
}

rootProject.name = "FtcAutoTune"

include(":core")
include(":ftc")
