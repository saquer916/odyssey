# Contributing to Odyssey

Thanks for wanting to help! Odyssey is a young project run by a beginner, so help of every size
is welcome, and so are beginners. If something here is unclear, that's a bug in this page; open an
issue and say so.

## Ways to help

- **Report a problem.** Something doesn't build, the robot does something odd, the docs are wrong:
  [open an issue](https://github.com/saquer916/odyssey/issues/new/choose) with the *Bug report* form.
- **Suggest an idea.** Use the *Feature request* form. The [roadmap](../README.md#roadmap) lists what's
  most needed.
- **Test on a real robot.** This is the most valuable thing right now. Odyssey has only been tuned in
  the simulator. Run `Drive Sign Check` and `SamplePath` on your robot and tell us what happened
  (the bug report form has room for it).
- **Improve the docs.** Fixing an unclear sentence is a perfectly good first pull request.
- **Write code.** Pick an issue labelled `good first issue` or `help wanted`, or one from the roadmap.

## Before you write code

Comment on the issue you want to work on (or open one) so nobody else does the same thing, and so
we can agree on the approach before you put time in. Small fixes like typos can skip this.

## How a contribution works

If you've never made a pull request, this is the whole process.

1. **Fork** the repo: the *Fork* button on GitHub makes your own copy under your account.
2. **Clone** your fork and open it in Android Studio (the same setup used for FTC robot code).
3. **Make a branch** off `test` for your change. Name it after what it does, e.g.
   `fix/heading-wraparound` or `docs/tuning-guide`.
   ```
   git checkout test
   git pull
   git checkout -b fix/heading-wraparound
   ```
4. **Make your change**, and keep it to one thing. Two unrelated fixes are two pull requests.
5. **Check it** (see [Testing your change](#testing-your-change)).
6. **Push** the branch to your fork and open a **pull request into `test`**.
   On the "Open a pull request" page, set **base: `test`**. GitHub suggests `main` by default, so
   change it.
7. **Review.** The simulator runs automatically on your pull request. A maintainer reads the change
   and may ask questions or for changes. That's normal and not a rejection: push more commits to the
   same branch and the pull request updates.
8. **Merge.** Once it's approved, a maintainer merges it into `test`. After it's been tried on a
   robot, `test` is merged into `main`.

### Branches

| Branch | Purpose |
|---|---|
| `main` | Stable. Only updated from `test` after testing on a robot. Don't open pull requests into it. |
| `test` | Where finished changes come together. **Open pull requests here.** |
| your branch | One change, e.g. `fix/…`, `feat/…`, `docs/…`. Deleted after it's merged. |

## Testing your change

From the repo root:

```
./gradlew :odyssey-core:test          # library unit tests
./gradlew :odyssey-sim:test           # simulator tests (they run FirstTest and DriveSignCheck)
./gradlew :odyssey-sim:run --args="TeamCode/src/main/java/org/firstinspires/ftc/teamcode/odyssey/opmodes/SamplePath.java"
```

The last command simulates an OpMode and writes `odyssey-sim/out/SamplePath/report.html`, which shows
how closely the robot followed the path. If your change affects how the robot drives, say in your
pull request how the simulator results changed, before vs. after.

No computer set up for this? Push your branch, and the **Actions** tab on your fork runs the same
checks.

**If you test on a real robot:** start with the robot on blocks or with lots of room, keep a hand
on the stop button, and run `Drive Sign Check` before any path.

## Writing code

- **Match the code around it.** Same naming, same comment style, same level of detail.
- **Units:** millimetres, seconds, radians. Robot frame: +X forward, +Y left, counter-clockwise
  positive (the Pinpoint's convention).
- **`odyssey-core` stays plain Java** with no FTC SDK imports, so it can be unit-tested on a computer.
  Robot-specific code goes in `TeamCode/.../odyssey/`.
- **Add a test** in `odyssey-core/src/test` for a library fix, ideally one that fails without the fix.
- **Comments explain why**, not what: the unit, the assumption, the reason for a number.
- **Don't change measured robot constants** (`kS`, `kV`, `kA`, `lX`, `lY`) unless you've measured them.

## Commit messages

Short first line saying what the commit does, in lowercase like the existing history
(`fix NaN drive signal at doubled bezier control points`). If it needs explaining, leave a blank line
and say why underneath.

## Code of conduct

Be kind and assume good intent; everyone here is learning. See [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md).

## License

Odyssey's own code is under the MIT License (see [LICENSE-ODYSSEY](../LICENSE-ODYSSEY)). By
contributing, you agree that your contribution is licensed the same way.
