import importlib.util
import unittest
from pathlib import Path


spec = importlib.util.spec_from_file_location(
    "update_doc_versions", Path(__file__).with_name("update-doc-versions.py")
)
updater = importlib.util.module_from_spec(spec)
spec.loader.exec_module(updater)


class UpdateDocVersionsTests(unittest.TestCase):
    def test_stale_examples_and_dependency_options(self):
        original = '''~~~toml title="Cargo.toml"
[dependencies]
mt_pubsub = "0.9.0"
mt_pubsub = { version = "0.14.1", default-features = false }
  mt_rat = { features = ["foo"], version = '0.9' }
tokio = { version = "1.49", features = ["full"] }
other_mt_pubsub = "0.9.0"
~~~
'''
        expected = original.replace('"0.9.0"', '"0.15.0"', 1).replace(
            '"0.14.1"', '"0.15.0"'
        ).replace("'0.9'", "'0.15.0'")
        self.assertEqual(
            updater.update_examples(original, {"mt_pubsub", "mt_rat"}, "0.15.0"),
            expected,
        )

    def test_prerelease_and_repeated_update(self):
        expected = 'mt_pubsub = "0.15.0-rc.1+build.2"\n'
        for original in ['mt_pubsub = "0.9.0"\n', expected]:
            self.assertEqual(
                updater.update_examples(original, {"mt_pubsub"}, "0.15.0-rc.1+build.2"),
                expected,
            )


if __name__ == "__main__":
    unittest.main()
